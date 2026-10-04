#pragma once

#include <memory>

#include <opencv2/features2d.hpp>

#include "stereo_vslam/core/frame.h"

namespace stereo_vslam {

/**
 * FeatureExtractor
 * ORB detection on the left image of a frame.
 */
class FeatureExtractor {
   public:
    using Ptr = std::shared_ptr<FeatureExtractor>;

    /** @brief Build an ORB detector that keeps up to @p num_features. */
    explicit FeatureExtractor(int num_features);

    /**
     * @brief Detect ORB features away from the existing left features and append
     * them.
     * @return Number of new features.
     */
    int detectNew(const Frame::Ptr &frame);

   private:
    cv::Ptr<cv::ORB> detector_;
};

}  // namespace stereo_vslam
