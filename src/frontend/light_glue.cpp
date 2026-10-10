#include "stereo_vslam/frontend/light_glue.h"

#include "stereo_vslam/frontend/ort_session.h"

#include <algorithm>
#include <array>
#include <stdexcept>

namespace stereo_vslam {
namespace {

std::vector<float> normalizeKeypoints(const std::vector<cv::Point2f> &keypoints,
                                      const cv::Size &size) {
    std::vector<float> normalized(keypoints.size() * 2);
    const float shift_x = static_cast<float>(size.width) * 0.5f;
    const float shift_y = static_cast<float>(size.height) * 0.5f;
    const float scale = std::max(size.width, size.height) * 0.5f;
    if (scale <= 0.f) {
        throw std::runtime_error("LightGlue received an empty image size");
    }
    for (size_t i = 0; i < keypoints.size(); ++i) {
        normalized[i * 2] = (keypoints[i].x - shift_x) / scale;
        normalized[i * 2 + 1] = (keypoints[i].y - shift_y) / scale;
    }
    return normalized;
}

}  // namespace

LightGlue::LightGlue(const std::string &model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "lightglue"),
      session_(makeOrtSession(env_, model_path)),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {}

std::vector<Match> LightGlue::match(const Features &features0, const cv::Size &size0,
                                    const Features &features1,
                                    const cv::Size &size1) const {
    if (features0.keypoints.empty() || features1.keypoints.empty()) {
        return {};
    }
    if (features0.descriptor_dim != features1.descriptor_dim) {
        throw std::runtime_error("LightGlue descriptor dimensions do not match");
    }

    const int descriptor_dim = features0.descriptor_dim;
    std::vector<float> keypoints0 = normalizeKeypoints(features0.keypoints, size0);
    std::vector<float> keypoints1 = normalizeKeypoints(features1.keypoints, size1);
    std::vector<float> descriptors0 = features0.descriptors;
    std::vector<float> descriptors1 = features1.descriptors;

    const int64_t count0 = static_cast<int64_t>(features0.keypoints.size());
    const int64_t count1 = static_cast<int64_t>(features1.keypoints.size());
    const std::array<int64_t, 3> keypoint_shape0{1, count0, 2};
    const std::array<int64_t, 3> keypoint_shape1{1, count1, 2};
    const std::array<int64_t, 3> descriptor_shape0{1, count0, descriptor_dim};
    const std::array<int64_t, 3> descriptor_shape1{1, count1, descriptor_dim};

    std::array<Ort::Value, 4> inputs = {
        Ort::Value::CreateTensor<float>(memory_info_, keypoints0.data(),
                                        keypoints0.size(), keypoint_shape0.data(),
                                        keypoint_shape0.size()),
        Ort::Value::CreateTensor<float>(memory_info_, keypoints1.data(),
                                        keypoints1.size(), keypoint_shape1.data(),
                                        keypoint_shape1.size()),
        Ort::Value::CreateTensor<float>(memory_info_, descriptors0.data(),
                                        descriptors0.size(), descriptor_shape0.data(),
                                        descriptor_shape0.size()),
        Ort::Value::CreateTensor<float>(memory_info_, descriptors1.data(),
                                        descriptors1.size(), descriptor_shape1.data(),
                                        descriptor_shape1.size()),
    };

    const char *input_names[] = {"kpts0", "kpts1", "desc0", "desc1"};
    const char *output_names[] = {"matches0", "mscores0"};
    auto outputs =
        session_.Run(Ort::RunOptions{nullptr}, input_names, inputs.data(),
                     inputs.size(), output_names, 2);

    const auto match_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    const int64_t match_count = match_shape.empty() ? 0 : match_shape[0];
    const int64_t *pairs = outputs[0].GetTensorData<int64_t>();
    const float *scores = outputs[1].GetTensorData<float>();

    std::vector<Match> matches;
    matches.reserve(static_cast<size_t>(match_count));
    for (int64_t i = 0; i < match_count; ++i) {
        Match match;
        match.index0 = static_cast<int>(pairs[i * 2]);
        match.index1 = static_cast<int>(pairs[i * 2 + 1]);
        match.score = scores[i];
        matches.push_back(match);
    }
    return matches;
}

}  // namespace stereo_vslam
