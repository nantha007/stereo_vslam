#pragma once

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace stereo_vslam {

struct Features {
    std::vector<cv::Point2f> keypoints;
    std::vector<float> scores;
    std::vector<float> descriptors;
    int descriptor_dim = 256;
};

/**
 * SuperPoint
 * ONNX SuperPoint detector and descriptor.
 */
class SuperPoint {
   public:
    explicit SuperPoint(const std::string &model_path);

    /** @brief Keypoints, scores, and 256-d descriptors for @p image. */
    Features extract(const cv::Mat &image) const;

   private:
    Ort::Env env_;
    mutable Ort::Session session_;
    Ort::MemoryInfo memory_info_;
};
}  // namespace stereo_vslam
