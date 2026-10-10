#include "stereo_vslam/visual_slam.h"

#include <glog/logging.h>

#include "stereo_vslam/core/config.h"
#include "stereo_vslam/core/scoped_log_timer.h"
#include "stereo_vslam/frontend/feature_extractor.h"
#include "stereo_vslam/frontend/keyframe_policy.h"
#include "stereo_vslam/frontend/light_glue.h"
#include "stereo_vslam/frontend/stereo_matcher.h"
#include "stereo_vslam/loop/place_recognizer.h"

namespace stereo_vslam {

VisualSLAM::VisualSLAM(const std::string &config_path) {
    SlamConfig config(config_path);

    dataset_ = Dataset::Ptr(new Dataset(config.dataset.dataset_path));
    Camera::Ptr camera_left = dataset_->getCamera(0);
    Camera::Ptr camera_right = dataset_->getCamera(1);

    map_ = Map::Ptr(new Map);

    auto extractor = std::make_shared<FeatureExtractor>(config.feature_extractor);
    LightGlue::Ptr light_glue;
    if (config.feature_extractor.type == FeatureType::SuperPoint) {
        light_glue =
            std::make_shared<LightGlue>(config.feature_extractor.lightglue_model);
    }
    auto stereo = std::make_shared<StereoMatcher>(
        camera_left, camera_right, config.tracking.far_landmark_filter);
    tracker_ = Tracker::Ptr(new Tracker(
        config.tracking, camera_left, extractor, stereo,
        KeyframePolicy(config.tracking.keyframe)));

    place_recognizer_ = std::make_shared<PlaceRecognizer>(
        config.loop_closure.vocabulary_path, config.feature_extractor.type);
    local_mapper_ = std::make_shared<LocalMapper>(
        config.local_mapper, map_, camera_left, camera_right);

    if (config.viewer.enabled) {
        viewer_ = Viewer::Ptr(new Viewer(map_));
    }
    LoopClosure::LoopClosedCallback on_loop;
    if (viewer_) {
        std::weak_ptr<Viewer> weak_viewer = viewer_;
        on_loop = [weak_viewer](const PoseMap &before, const PoseMap &after) {
            if (auto viewer = weak_viewer.lock()) {
                viewer->onLoopClosed(before, after);
            }
        };
    }
    loop_closure_ = LoopClosure::Ptr(new LoopClosure(
        config.loop_closure, map_, camera_left, on_loop, light_glue));
    if (viewer_) viewer_->start();
    loop_closure_->start();
}

VisualSLAM::~VisualSLAM() {
    if (loop_closure_) {
        loop_closure_->stop();
    }
    if (viewer_) {
        viewer_->stop();
    }
}

void VisualSLAM::run() {
    while (1) {
        LOG(INFO) << "SLAM is running";
        if (processFrame() == false) {
            break;
        }
    }

    loop_closure_->stop();
    if (viewer_) viewer_->stop();

    LOG(INFO) << "SLAM exited";
}

bool VisualSLAM::processFrame() {
    if (dataset_ == nullptr) return false;
    Frame::Ptr new_frame = dataset_->nextFrame();
    if (new_frame == nullptr) return false;

    ScopedLogTimer frame_timer("processFrame");
    auto state = map_->load();
    TrackResult result = tracker_->trackFrame(new_frame, state);
    bool inserted = false;
    if (!result.is_new_keyframe) {
        tracker_->commitTrack();
    } else {
        // Fill BoW before the keyframe is reachable from a published snapshot,
        // so loop closure never reads it while it is still being written.
        place_recognizer_->computeBoW(*new_frame);
        if (local_mapper_->insertKeyframe(state, new_frame, result)) {
            tracker_->commitTrack();
            inserted = true;
        } else {
            tracker_->discardTrackAttempt();
            LOG(INFO) << "Skipping this frame";
        }
    }

    if (result.is_pose_ready && viewer_) {
        ScopedLogTimer viewer_timer("onFrameTracked");
        viewer_->onFrameTracked(new_frame);
    }

    if (inserted) {
        local_mapper_->runLocalBA();
        loop_closure_->submitKeyframe(new_frame);
        if (viewer_) {
            ScopedLogTimer map_timer("onMapUpdated");
            viewer_->onMapUpdated();
        }
    }
    return true;
}

}  // namespace stereo_vslam
