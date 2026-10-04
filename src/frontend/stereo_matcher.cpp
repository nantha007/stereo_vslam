#include "stereo_vslam/frontend/stereo_matcher.h"

#include <cmath>

#include <glog/logging.h>

#include <opencv2/video/tracking.hpp>

#include "stereo_vslam/core/conversion.h"
#include "stereo_vslam/core/feature.h"
#include "stereo_vslam/core/scoped_log_timer.h"

namespace stereo_vslam {

StereoMatcher::StereoMatcher(Camera::Ptr camera_left, Camera::Ptr camera_right,
                             const FarLandmarkFilterConfig &far_landmark_filter)
    : camera_left_(std::move(camera_left)),
      camera_right_(std::move(camera_right)),
      far_landmark_filter_(far_landmark_filter) {}

int StereoMatcher::matchRight(const Frame::Ptr &frame, const MapState &state) {
    ScopedLogTimer timer("matchRight");
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> points_left, points_right;
    for (auto &feature : frame->features_left) {
        points_left.push_back(feature->keypoint.pt);
        auto landmark = feature->getLandmark();
        if (landmark) {
            // use projected points as initial guess
            auto px = camera_right_->worldToPixel(landmarkPosition(state, *landmark),
                                                   frame->getPose());
            points_right.push_back(toPoint2f(px));
        } else {
            // use same pixel in left image
            points_right.push_back(feature->keypoint.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        frame->image_left, frame->image_right, points_left, points_right, status,
        error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_points = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint keypoint(points_right[i], 7);
            Feature::Ptr feature(new Feature(frame, keypoint));
            feature->is_on_left_image = false;
            frame->features_right.push_back(feature);
            num_good_points++;
        } else {
            frame->features_right.push_back(nullptr);
        }
    }
    LOG(INFO) << "Found " << num_good_points << " points in the right image.";
    return num_good_points;
}

std::vector<Landmark::Ptr> StereoMatcher::triangulate(const Frame::Ptr &frame) {
    ScopedLogTimer timer("triangulate");
    const SE3 T_wc = frame->getPose().inverse();
    const double fx = camera_left_->fx;
    const double fy = camera_left_->fy;
    const double cx = camera_left_->cx;
    const double cy = camera_left_->cy;
    const double baseline = camera_right_->baseline;
    std::vector<Landmark::Ptr> new_landmarks;
    for (size_t i = 0; i < frame->features_left.size(); ++i) {
        const auto &feature_left = frame->features_left[i];
        const auto &feature_right = frame->features_right[i];
        if (feature_left->getLandmark() || feature_right == nullptr) continue;
        // Depth is fx * baseline / disparity, in the left camera frame.
        const Vec2 pt_left = toVec2(feature_left->keypoint.pt);
        const Vec2 pt_right = toVec2(feature_right->keypoint.pt);
        const double disparity = pt_left.x() - pt_right.x();
        if (disparity <= 0.0 || fx <= 0.0 || fy <= 0.0 || baseline <= 0.0) {
            continue;
        }

        const double depth = fx * baseline / disparity;
        if (!std::isfinite(depth) || depth <= 0.0) continue;

        Vec3 p_c;
        p_c << (pt_left.x() - cx) / fx * depth,
            (pt_left.y() - cy) / fy * depth, depth;
        if (far_landmark_filter_.isFar(p_c)) continue;

        auto new_landmark = Landmark::create();
        new_landmark->setRefKeyframeId(frame->keyframe_id);
        new_landmark->setPosition(T_wc * p_c);
        feature_left->setLandmark(new_landmark);
        feature_right->setLandmark(new_landmark);
        new_landmarks.push_back(new_landmark);
    }
    LOG(INFO) << "Created " << new_landmarks.size() << " new landmarks";
    return new_landmarks;
}

}  // namespace stereo_vslam
