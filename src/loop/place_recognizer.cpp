#include "stereo_vslam/loop/place_recognizer.h"

#include <glog/logging.h>

#include "stereo_vslam/core/feature.h"
#include "stereo_vslam/core/landmark.h"
#include "stereo_vslam/core/scoped_log_timer.h"

namespace stereo_vslam {

PlaceRecognizer::PlaceRecognizer(const std::string &vocabulary_path)
    : vocabulary_(std::make_unique<fbow::Vocabulary>()) {
    vocabulary_->readFromFile(vocabulary_path);
    CHECK(vocabulary_->isValid()) << "Failed to load FBoW vocabulary: "
                                  << vocabulary_path;
    LOG(INFO) << "Loaded FBoW vocabulary from " << vocabulary_path
              << " (descriptor=" << vocabulary_->getDescName() << ")";
}

void PlaceRecognizer::computeBoW(Frame &frame) const {
    ScopedLogTimer timer("computeBoW");
    if (!frame.bow.empty()) return;

    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Mat> rows;
    std::vector<std::weak_ptr<Landmark>> landmarks;
    keypoints.reserve(frame.features_left.size());
    rows.reserve(frame.features_left.size());
    landmarks.reserve(frame.features_left.size());

    for (const auto &feature : frame.features_left) {
        if (!feature || feature->descriptor.empty()) continue;
        keypoints.push_back(feature->keypoint);
        rows.push_back(feature->descriptor);
        auto landmark = feature->getLandmark();
        landmarks.push_back(landmark ? std::weak_ptr<Landmark>(landmark)
                                     : std::weak_ptr<Landmark>{});
    }

    if (rows.empty()) {
        LOG(WARNING) << "No ORB descriptors for keyframe "
                     << frame.keyframe_id;
        return;
    }

    cv::Mat descriptors;
    cv::vconcat(rows, descriptors);
    frame.bow_keypoints = std::move(keypoints);
    frame.bow_descriptors = descriptors;
    frame.bow_landmarks = std::move(landmarks);
    frame.bow = vocabulary_->transform(descriptors);

    LOG(INFO) << "Computed FBoW for keyframe " << frame.keyframe_id
              << " with " << frame.bow.size() << " words from "
              << descriptors.rows << " ORB descriptors";
}

}  // namespace stereo_vslam
