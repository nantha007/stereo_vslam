#include "stereo_vslam/frontend/tracker.h"

#include <glog/logging.h>
#include <opencv2/video/tracking.hpp>

#include "stereo_vslam/backend/optimizer.h"
#include "stereo_vslam/core/conversion.h"
#include "stereo_vslam/core/feature.h"
#include "stereo_vslam/core/scoped_log_timer.h"

namespace stereo_vslam {

Tracker::Tracker(const TrackingConfig &config, Camera::Ptr camera_left,
                 FeatureExtractor::Ptr feature_extractor,
                 StereoMatcher::Ptr stereo_matcher,
                 const KeyframePolicy &keyframe_policy)
    : camera_left_(std::move(camera_left)),
      config_(config),
      feature_extractor_(std::move(feature_extractor)),
      stereo_matcher_(std::move(stereo_matcher)),
      keyframe_policy_(keyframe_policy) {}

TrackResult Tracker::trackFrame(Frame::Ptr frame,
                                const std::shared_ptr<const MapState> &state) {
    if (state) applyPendingCorrection(*state);
    relative_motion_backup_ = relative_motion_;
    current_frame_ = frame;
    result_ = TrackResult();
    ScopedLogTimer timer("trackFrame", &current_frame_->is_keyframe);

    if (state) {
        if (!is_initialized_) {
            stereoInit(*state);
        } else {
            track(*state);
        }
    }

    return result_;
}

void Tracker::commitTrack() {
    last_frame_ = current_frame_;
    if (result_.is_new_keyframe && current_frame_ && current_frame_->is_keyframe) {
        last_keyframe_ = current_frame_;
        is_initialized_ = true;
    }
}

void Tracker::discardTrackAttempt() {
    relative_motion_ = relative_motion_backup_; 
}

void Tracker::applyPendingCorrection(const MapState &state) {
    if (last_keyframe_) {
        auto pose = state.keyframe_poses.find(last_keyframe_->keyframe_id);
        if (pose != state.keyframe_poses.end()) {
            last_keyframe_->setPose(pose->second);
        }
    }

    if (state.epoch == applied_epoch_) return;

    if (last_frame_ && !last_frame_->is_keyframe) {
        const SE3 delta = applied_correction_.inverse() * state.accumulated_correction;
        last_frame_->setPose(last_frame_->getPose() * delta);
    }
    applied_epoch_ = state.epoch;
    applied_correction_ = state.accumulated_correction;
}

void Tracker::track(const MapState &state) {
    ScopedLogTimer timer("track");
    SE3 predicted = current_frame_->getPose();
    if (last_frame_) {
        predicted = relative_motion_ * last_frame_->getPose();
        current_frame_->setPose(predicted);
    }

    trackFeaturesFromLastFrame(state);
    unlinkFarLandmarks(state);
    std::pair<int, SE3> estimate = estimateCurrentPose(state);
    tracking_inliers_ = estimate.first;

    if (tracking_inliers_ <= config_.min_pose_inliers) {
        // The estimate is underconstrained and can jump hundreds of meters.
        // Keep the motion model and do not insert this frame as a keyframe.
        current_frame_->setPose(predicted);
        for (auto &feature : current_frame_->features_left) {
            if (feature) feature->setOutlier(false);
        }
        LOG(INFO) << "Tracking inliers " << tracking_inliers_ << " below "
                  << config_.min_pose_inliers << ", keep motion-model pose";
        result_.is_pose_ready = true;
        return;
    }

    current_frame_->setPose(estimate.second);
    for (auto &feature : current_frame_->features_left) {
        if (feature && feature->isOutlier()) {
            feature->resetLandmark();
            feature->setOutlier(false);
        }
    }

    tryMakeKeyframe(state);
    relative_motion_ = current_frame_->getPose() * last_frame_->getPose().inverse();

    result_.is_pose_ready = true;
}

bool Tracker::tryMakeKeyframe(const MapState &state) {
    ScopedLogTimer timer("tryMakeKeyframe");
    const SE3 T_cw_current = current_frame_->getPose();
    const SE3 T_cw_last_keyframe =
        last_keyframe_ ? last_keyframe_->getPose() : T_cw_current;
    if (!keyframe_policy_.needsKeyframe(tracking_inliers_, T_cw_current,
                                        T_cw_last_keyframe)) {
        return false;
    }
    const double rotation_deg =
        KeyframePolicy::rotationDeg(T_cw_current, T_cw_last_keyframe);
    // Assign the id before triangulation stores it on each new landmark.
    result_.last_keyframe = last_keyframe_;
    current_frame_->markAsKeyframe();

    LOG(INFO) << "Set frame " << current_frame_->id << " as keyframe "
              << current_frame_->keyframe_id << " (inliers "
              << tracking_inliers_ << ", rotation " << rotation_deg
              << " deg)";

    feature_extractor_->detectNew(current_frame_);
    stereo_matcher_->matchRight(current_frame_, state);
    result_.new_landmarks = stereo_matcher_->triangulate(current_frame_);
    result_.is_new_keyframe = true;
    return true;
}

std::pair<int, SE3> Tracker::estimateCurrentPose(const MapState &state) {
    ScopedLogTimer timer("estimateCurrentPose");
    SE3 estimated;
    const int inliers =
        optimizer::optimizePose(*current_frame_, *camera_left_, state, &estimated);
    return {inliers, estimated};
}

int Tracker::trackFeaturesFromLastFrame(const MapState &state) {
    ScopedLogTimer timer("trackFeaturesFromLastFrame");
    // use LK flow to track points into the current left image
    std::vector<cv::Point2f> points_last, points_current;
    for (auto &feature : last_frame_->features_left) {
        if (feature->getLandmark()) {
            // use project point
            auto landmark = feature->getLandmark();
            auto px = camera_left_->worldToPixel(landmarkPosition(state, *landmark),
                                                 current_frame_->getPose());
            points_last.push_back(feature->keypoint.pt);
            points_current.push_back(toPoint2f(px));
        } else {
            points_last.push_back(feature->keypoint.pt);
            points_current.push_back(feature->keypoint.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->image_left, current_frame_->image_left, points_last,
        points_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_points = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint keypoint(points_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, keypoint));
            feature->setLandmark(last_frame_->features_left[i]->getLandmark());
            current_frame_->features_left.push_back(feature);
            num_good_points++;
        }
    }

    LOG(INFO) << "Found " << num_good_points << " points in the current image.";
    return num_good_points;
}

bool Tracker::stereoInit(const MapState &state) {
    ScopedLogTimer timer("stereoInit");
    feature_extractor_->detectNew(current_frame_);
    stereo_matcher_->matchRight(current_frame_, state);

    // Assign the id before triangulation stores it on each landmark.
    // last_keyframe_ and is_initialized_ wait until commitTrack.
    current_frame_->markAsKeyframe();
    result_.new_landmarks = stereo_matcher_->triangulate(current_frame_);
    result_.is_new_keyframe = true;
    result_.is_pose_ready = true;

    LOG(INFO) << "Initial map created with " << result_.new_landmarks.size()
              << " landmarks";
    return true;
}

void Tracker::unlinkFarLandmarks(const MapState &state) {
    ScopedLogTimer timer("unlinkFarLandmarks");
    if (!config_.far_landmark_filter.enabled || !current_frame_) return;
    const SE3 T_cw = current_frame_->getPose();
    int num_removed = 0;

    auto unlink = [&](const Feature::Ptr &feature) {
        if (!feature) return;
        auto landmark = feature->getLandmark();
        if (!landmark) return;
        if (!config_.far_landmark_filter.isFar(
                T_cw * landmarkPosition(state, *landmark)))
            return;
        // These features are not observations yet. Observations are registered
        // only when LocalMapper inserts a keyframe, so removeObservation would
        // miss this feature and leave the landmark link in place.
        feature->resetLandmark();
        ++num_removed;
    };

    for (auto &feature : current_frame_->features_left) unlink(feature);
    for (auto &feature : current_frame_->features_right) unlink(feature);

    if (num_removed > 0) {
        LOG(INFO) << "Unlinked " << num_removed << " features farther than "
                  << config_.far_landmark_filter.max_distance_m << " m";
    }
}

}  // namespace stereo_vslam
