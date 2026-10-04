#include "stereo_vslam/frontend/feature_extractor.h"

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include "stereo_vslam/core/feature.h"
#include "stereo_vslam/core/scoped_log_timer.h"

namespace stereo_vslam {

FeatureExtractor::FeatureExtractor(int num_features)
    : detector_(cv::ORB::create(num_features)) {}

int FeatureExtractor::detectNew(const Frame::Ptr &frame) {
    ScopedLogTimer timer("detectNew");
    cv::Mat mask(frame->image_left.size(), CV_8UC1, 255);
    for (auto &feature : frame->features_left) {
        cv::rectangle(mask, feature->keypoint.pt - cv::Point2f(10, 10),
                      feature->keypoint.pt + cv::Point2f(10, 10), 0, cv::FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detector_->detectAndCompute(frame->image_left, mask, keypoints,
                                descriptors);
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
