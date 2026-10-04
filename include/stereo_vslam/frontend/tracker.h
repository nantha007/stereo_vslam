#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "stereo_vslam/core/camera.h"
#include "stereo_vslam/core/config.h"
#include "stereo_vslam/core/types.h"
#include "stereo_vslam/core/frame.h"
#include "stereo_vslam/core/landmark.h"
#include "stereo_vslam/core/map.h"
#include "stereo_vslam/frontend/feature_extractor.h"
#include "stereo_vslam/frontend/keyframe_policy.h"
#include "stereo_vslam/frontend/stereo_matcher.h"

namespace stereo_vslam {

/// Outcome of tracking one frame
struct TrackResult {
    /// The frame has a pose worth showing
    bool is_pose_ready = false;
    /// The frame became a keyframe and must be inserted into the map
    bool is_new_keyframe = false;
    /// Keyframe before the new one; nullptr for the first keyframe
    Frame::Ptr last_keyframe;
    /// Landmarks triangulated on the new keyframe, not yet in the map
    std::vector<Landmark::Ptr> new_landmarks;
};

/**
 * Tracker
 * Estimates the current frame pose from the previous frame and decides
 * whether it becomes a keyframe. When it does, new features are detected
 * and triangulated on it; inserting it into the map is left to the caller.
 */
class Tracker {
   public:
    using Ptr = std::shared_ptr<Tracker>;

    /** @brief Build a tracker for the left camera. */
    Tracker(const TrackingConfig &config, Camera::Ptr camera_left,
            FeatureExtractor::Ptr feature_extractor,
            StereoMatcher::Ptr stereo_matcher,
            const KeyframePolicy &keyframe_policy);

    /**
     * @brief Track @p frame against @p state without committing bookkeeping.
     *
     * Call commitTrack() after the frame is kept, or discardTrackAttempt()
     * when a keyframe publish loses the revision race.
     */
    TrackResult trackFrame(Frame::Ptr frame,
                           const std::shared_ptr<const MapState> &state);

    /** @brief Keep last_frame_, and the keyframe pointer when this attempt inserted one. */
    void commitTrack();

    /** @brief Restore the motion model from before this attempt. last_frame_ stays. */
    void discardTrackAttempt();

   private:
    /**
     * @brief Copy the last keyframe pose from @p state so a local BA update is
     * visible.
     *
     * A non-keyframe last_frame_ is corrected when the map epoch moves.
     */
    void applyPendingCorrection(const MapState &state);

    /** @brief Track the current frame from the motion model of the previous frame. */
    void track(const MapState &state);

    /**
     * @brief Track the last frame's features into the current left image.
     * @return Number of tracked points.
     */
    int trackFeaturesFromLastFrame(const MapState &state);

    /**
     * @brief Estimate the current frame pose.
     * @return Inlier count and the estimated pose.
     */
    std::pair<int, SE3> estimateCurrentPose(const MapState &state);

    /**
     * @brief Turn the current frame into a keyframe when the policy asks for one.
     *
     * Assigns the keyframe id, detects new features, matches them in the right
     * image, and triangulates. The caller inserts the keyframe into the map.
     * @return True if the frame became a keyframe.
     */
    bool tryMakeKeyframe(const MapState &state);

    /**
     * @brief Initialize from the stereo pair in the current frame.
     * @return True on success.
     */
    bool stereoInit(const MapState &state);

    /** @brief Unlink landmarks on the current frame that are farther than the limit. */
    void unlinkFarLandmarks(const MapState &state);

    // data
    uint64_t applied_epoch_ = 0;
    SE3 applied_correction_;

    bool is_initialized_ = false;
    TrackResult result_;  // filled while tracking the current frame

    Frame::Ptr current_frame_ = nullptr;  // current frame
    Frame::Ptr last_frame_ = nullptr;     // last frame
    Frame::Ptr last_keyframe_ = nullptr;  // last inserted keyframe
    Camera::Ptr camera_left_ = nullptr;   // left camera

    SE3 relative_motion_;  // relative motion w.r.t. last frame, used as pose initial guess
    SE3 relative_motion_backup_;

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

    TrackingConfig config_;

    // utilities
    FeatureExtractor::Ptr feature_extractor_;
    StereoMatcher::Ptr stereo_matcher_;
    KeyframePolicy keyframe_policy_;
};

}  // namespace stereo_vslam
