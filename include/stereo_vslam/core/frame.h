#pragma once

#include <memory>
#include <vector>

#include <fbow/fbow.h>
#include <opencv2/core.hpp>

#include "stereo_vslam/core/camera.h"
#include "stereo_vslam/core/feature.h"
#include "stereo_vslam/core/types.h"

namespace stereo_vslam {

// forward declare
struct Landmark;

/**
 * Frame
 * Each frame gets a unique id; keyframes get a keyframe id.
 * pose_ is the private pose of this frame. A published keyframe's pose lives
 * in the map snapshot. The tracker copies that pose back before it uses the
 * keyframe again. Images, features, and BoW are written on the main thread.
 */
struct Frame {
   public:
    using Ptr = std::shared_ptr<Frame>;

    Id id = 0;           // id of this frame
    Id keyframe_id = 0;  // id of keyframe
    bool is_keyframe = false;  // whether this is a keyframe
    cv::Mat image_left, image_right;  // stereo images

    // extracted features in left image
    std::vector<Feature::Ptr> features_left;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<Feature::Ptr> features_right;

    // FBoW bag-of-words vector, filled by PlaceRecognizer for keyframes
    fbow::fBow bow;
    // Left-image keypoints/descriptors used for BoW and loop matching
    std::vector<cv::KeyPoint> bow_keypoints;
    cv::Mat bow_descriptors;
    // Landmark of the left-image feature that produced each descriptor
    std::vector<std::weak_ptr<Landmark>> bow_landmarks;

    /** @brief Create an empty frame. The id is assigned by create(). */
    Frame() {}

    /** @brief Private pose in T_cw. */
    SE3 getPose() const { return pose_; }

    /** @brief Set the private pose. @p new_pose is T_cw. */
    void setPose(const SE3 &new_pose) { pose_ = new_pose; }

    /** @brief Mark as a keyframe and assign a keyframe id. */
    void markAsKeyframe();

    /**
     * @brief Drop a tracking attempt that was not inserted.
     *
     * Images and the frame id stay. Pose is left for the next attempt to
     * overwrite.
     */
    void clear();

    /** @brief Release the images and features. Pose and ids stay. */
    void releaseImagesAndFeatures();

    /** @brief Create a frame and assign its id. */
    static Ptr create();

   private:
    SE3 pose_;  // pose in T_cw form
};

}  // namespace stereo_vslam
