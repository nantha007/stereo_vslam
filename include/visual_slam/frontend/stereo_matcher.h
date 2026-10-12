#pragma once

#include <memory>
#include <vector>

#include "visual_slam/core/camera.h"
#include "visual_slam/core/config.h"
#include "visual_slam/core/frame.h"
#include "visual_slam/core/landmark.h"
#include "visual_slam/core/map.h"

namespace visual_slam {

/**
 * StereoMatcher
 * Left-to-right feature matching and stereo triangulation on a rectified pair.
 */
class StereoMatcher {
   public:
    using Ptr = std::shared_ptr<StereoMatcher>;

    /** @brief Build a matcher for a rectified stereo pair. */
    StereoMatcher(Camera::Ptr camera_left, Camera::Ptr camera_right,
                  const FarLandmarkFilterConfig &far_landmark_filter);

    /**
     * @brief Track every left feature into the right image with LK flow.
     *
     * Fills frame->features_right index-aligned with features_left, nullptr
     * where tracking failed.
     * @return Number of matched features.
     */
    int matchRight(const Frame::Ptr &frame, const MapState &state);

    /**
     * @brief Triangulate left features that have a right match and no landmark.
     *
     * Links both features to the new landmark and sets the reference keyframe,
     * so frame->keyframe_id must already be assigned. Observations are not
     * registered and nothing is inserted into the map.
     * @return Landmarks created on this frame.
     */
    std::vector<Landmark::Ptr> triangulate(const Frame::Ptr &frame);

   private:
    Camera::Ptr camera_left_;
    Camera::Ptr camera_right_;
    FarLandmarkFilterConfig far_landmark_filter_;
};

}  // namespace visual_slam
