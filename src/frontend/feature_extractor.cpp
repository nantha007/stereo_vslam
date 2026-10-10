#include "stereo_vslam/frontend/feature_extractor.h"

#include <algorithm>
#include <cmath>

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include "stereo_vslam/core/feature.h"
#include "stereo_vslam/core/scoped_log_timer.h"
#include "stereo_vslam/frontend/superpoint.h"

namespace stereo_vslam {
namespace {

cv::Mat featureMask(const Frame &frame) {
    cv::Mat mask(frame.image_left.size(), CV_8UC1, 255);
    for (auto &feature : frame.features_left) {
        cv::rectangle(mask, feature->keypoint.pt - cv::Point2f(10, 10),
                      feature->keypoint.pt + cv::Point2f(10, 10), 0, cv::FILLED);
    }
    return mask;
}

}  // namespace

FeatureExtractor::FeatureExtractor(const FeatureExtractorConfig &config)
    : config_(config) {
    if (config_.type == FeatureType::SuperPoint) {
        superpoint_ = std::make_unique<SuperPoint>(config_.superpoint_model);
    } else {
        detector_ = cv::ORB::create(config_.num_features);
    }
}

FeatureExtractor::~FeatureExtractor() = default;

int FeatureExtractor::detectNew(const Frame::Ptr &frame) {
    ScopedLogTimer timer("detectNew");
    cv::Mat mask = featureMask(*frame);

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    if (superpoint_) {
        const Features extracted = superpoint_->extract(frame->image_left);
        struct Candidate {
            cv::KeyPoint keypoint;
            int index = 0;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(extracted.keypoints.size());
        for (size_t i = 0; i < extracted.keypoints.size(); ++i) {
            const cv::Point2f &pt = extracted.keypoints[i];
            const int x = static_cast<int>(std::lround(pt.x));
            const int y = static_cast<int>(std::lround(pt.y));
            if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows) continue;
            if (mask.at<uchar>(y, x) == 0) continue;
            cv::KeyPoint keypoint(pt, 7);
            keypoint.response =
                i < extracted.scores.size() ? extracted.scores[i] : 0.f;
            candidates.push_back(Candidate{keypoint, static_cast<int>(i)});
        }
        if (static_cast<int>(candidates.size()) > config_.num_features) {
            std::partial_sort(
                candidates.begin(), candidates.begin() + config_.num_features,
                candidates.end(), [](const Candidate &a, const Candidate &b) {
                    return a.keypoint.response > b.keypoint.response;
                });
            candidates.resize(static_cast<size_t>(config_.num_features));
        }
        const int dim = extracted.descriptor_dim;
        int num_detected = 0;
        for (const auto &candidate : candidates) {
            cv::Mat descriptor(1, dim, CV_32F);
            if (dim > 0) {
                const float *src =
                    extracted.descriptors.data() +
                    static_cast<size_t>(candidate.index) * static_cast<size_t>(dim);
                std::copy(src, src + dim, descriptor.ptr<float>());
            }
            frame->features_left.push_back(Feature::Ptr(
                new Feature(frame, candidate.keypoint, descriptor)));
            num_detected++;
        }
        LOG(INFO) << "Detected " << num_detected << " new features";
        return num_detected;
    }

    detector_->detectAndCompute(frame->image_left, mask, keypoints, descriptors);
    int num_detected = 0;
    for (size_t i = 0; i < keypoints.size(); ++i) {
        cv::Mat descriptor =
            descriptors.empty() ? cv::Mat() : descriptors.row(i).clone();
        frame->features_left.push_back(
            Feature::Ptr(new Feature(frame, keypoints[i], descriptor)));
        num_detected++;
    }

    LOG(INFO) << "Detected " << num_detected << " new features";
    return num_detected;
}

}  // namespace stereo_vslam
