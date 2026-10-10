#pragma once

#include "stereo_vslam/frontend/superpoint.h"

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace stereo_vslam {

struct Match {
    int index0 = 0;
    int index1 = 0;
    float score = 0.f;
};

/**
 * LightGlue
 * ONNX matcher for SuperPoint keypoints and descriptors.
 */
class LightGlue {
   public:
    using Ptr = std::shared_ptr<LightGlue>;

    explicit LightGlue(const std::string &model_path);

    /**
     * @brief Match two SuperPoint feature sets.
     * @param size0 Image size used to normalize @p features0.
     * @param size1 Image size used to normalize @p features1.
     */
    std::vector<Match> match(const Features &features0, const cv::Size &size0,
                             const Features &features1,
                             const cv::Size &size1) const;

   private:
    Ort::Env env_;
    mutable Ort::Session session_;
    Ort::MemoryInfo memory_info_;
};

}  // namespace stereo_vslam
