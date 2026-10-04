#pragma once

#include <memory>

#include <opencv2/core.hpp>

namespace stereo_vslam {

struct Frame;
struct Landmark;

/**
 * 2D feature point
 * Associated with a landmark after triangulation.
 * landmark_ and is_outlier_ are updated on the main thread once the owning
 * frame is in the map.
 */
struct Feature {
   public:
    using Ptr = std::shared_ptr<Feature>;

    std::weak_ptr<Frame> frame;  // frame that owns this feature
    cv::KeyPoint keypoint;       // 2D extraction keypoint
    cv::Mat descriptor;          // ORB descriptor (1x32, CV_8U)
    bool is_on_left_image = true;  // true if detected on the left image, false for the right

    /** @brief Landmark linked to this feature, or empty. */
    std::shared_ptr<Landmark> getLandmark() const { return landmark_.lock(); }

    /** @brief Link this feature to @p landmark. */
    void setLandmark(const std::shared_ptr<Landmark> &landmark) {
        landmark_ = landmark;
    }

    /** @brief Clear the landmark link. */
    void resetLandmark() { landmark_.reset(); }

    /** @brief True when pose optimization marked this feature as an outlier. */
    bool isOutlier() const { return is_outlier_; }

    /** @brief Mark or clear the outlier flag. */
    void setOutlier(bool outlier) { is_outlier_ = outlier; }

   public:
    /** @brief Create an empty feature. */
    Feature() {}

    /**
     * @brief Create a feature on @p frame.
     * @param descriptor ORB descriptor. Empty when not yet computed.
     */
    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &keypoint,
            const cv::Mat &descriptor = cv::Mat())
        : frame(frame), keypoint(keypoint), descriptor(descriptor) {}

   private:
    std::weak_ptr<Landmark> landmark_;
    bool is_outlier_ = false;
};
}  // namespace stereo_vslam
