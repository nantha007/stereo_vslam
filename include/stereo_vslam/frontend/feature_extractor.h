#pragma once

#include <memory>

#include <opencv2/features2d.hpp>

#include "stereo_vslam/core/config.h"
#include "stereo_vslam/core/frame.h"

namespace stereo_vslam {

class SuperPoint;

/**
 * FeatureExtractor
 * ORB or SuperPoint detection on the left image of a frame.
 */
class FeatureExtractor {
   public:
    using Ptr = std::shared_ptr<FeatureExtractor>;

    /** @brief Build the detector selected by @p config. */
    explicit FeatureExtractor(const FeatureExtractorConfig &config);
    ~FeatureExtractor();

    /**
     * @brief Detect features away from the existing left features and append
     * them.
     * @return Number of new features.
     */
    int detectNew(const Frame::Ptr &frame);

   private:
    FeatureExtractorConfig config_;
    cv::Ptr<cv::ORB> detector_;
    std::unique_ptr<SuperPoint> superpoint_;
};

}  // namespace stereo_vslam
