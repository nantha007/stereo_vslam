#include "stereo_vslam/frontend/superpoint.h"

#include "stereo_vslam/frontend/ort_session.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <stdexcept>

namespace stereo_vslam {
namespace {

cv::Mat toUnitGray(const cv::Mat &image) {
    if (image.empty()) {
        throw std::runtime_error("SuperPoint received an empty image");
    }

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 1) {
        gray = image;
    } else {
        throw std::runtime_error("SuperPoint expects a grayscale or BGR image");
    }

    cv::Mat unit;
    if (gray.type() == CV_32FC1) {
        unit = gray.isContinuous() ? gray : gray.clone();
    } else {
        gray.convertTo(unit, CV_32F, 1.0 / 255.0);
    }
    return unit;
}

}  // namespace

SuperPoint::SuperPoint(const std::string &model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "superpoint"),
      session_(makeOrtSession(env_, model_path)),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {}

Features SuperPoint::extract(const cv::Mat &image) const {
    cv::Mat unit = toUnitGray(image);
    const int64_t height = unit.rows;
    const int64_t width = unit.cols;
    const std::array<int64_t, 4> input_shape{1, 1, height, width};

    Ort::Value input = Ort::Value::CreateTensor<float>(
        memory_info_, unit.ptr<float>(), static_cast<size_t>(height * width),
        input_shape.data(), input_shape.size());

    const char *input_names[] = {"image"};
    const char *output_names[] = {"keypoints", "scores", "descriptors"};
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names, &input, 1,
                                output_names, 3);

    const auto keypoint_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    const auto score_shape = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
    const auto descriptor_shape = outputs[2].GetTensorTypeAndShapeInfo().GetShape();
    const int64_t count = keypoint_shape.size() >= 2 ? keypoint_shape[1] : 0;
    const int64_t descriptor_dim =
        descriptor_shape.size() >= 3 ? descriptor_shape.back() : 256;

    Features features;
    features.descriptor_dim = static_cast<int>(descriptor_dim);
    features.keypoints.reserve(static_cast<size_t>(count));
    features.scores.reserve(static_cast<size_t>(count));
    features.descriptors.resize(static_cast<size_t>(count * descriptor_dim));

    const float *keypoints = outputs[0].GetTensorData<float>();
    const float *scores = outputs[1].GetTensorData<float>();
    const float *descriptors = outputs[2].GetTensorData<float>();
    const int64_t score_count = score_shape.size() >= 2 ? score_shape[1] : count;
    for (int64_t i = 0; i < count; ++i) {
        features.keypoints.emplace_back(keypoints[i * 2], keypoints[i * 2 + 1]);
        features.scores.push_back(i < score_count ? scores[i] : 0.f);
    }
    if (count > 0 && descriptor_dim > 0) {
        std::copy(descriptors, descriptors + count * descriptor_dim,
                  features.descriptors.begin());
    }
    return features;
}

}  // namespace stereo_vslam
