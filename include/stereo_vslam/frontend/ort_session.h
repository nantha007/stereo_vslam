#pragma once

#include <onnxruntime_cxx_api.h>

#include <string>

#include <glog/logging.h>

namespace stereo_vslam {

/** @brief CUDA session when the provider loads, otherwise CPU. */
inline Ort::Session makeOrtSession(Ort::Env &env, const std::string &model_path) {
    auto make_options = [](bool use_cuda) {
        Ort::SessionOptions options;
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        if (use_cuda) {
            OrtCUDAProviderOptions cuda;
            cuda.device_id = 0;
            options.AppendExecutionProvider_CUDA(cuda);
        }
        return options;
    };

    try {
        Ort::SessionOptions options = make_options(true);
        return Ort::Session(env, model_path.c_str(), options);
    } catch (const Ort::Exception &error) {
        LOG(WARNING) << "CUDA execution provider unavailable, using CPU: "
                     << error.what();
        Ort::SessionOptions options = make_options(false);
        return Ort::Session(env, model_path.c_str(), options);
    }
}

}  // namespace stereo_vslam
