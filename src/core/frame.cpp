#include "visual_slam/core/frame.h"
#include "visual_slam/core/feature.h"
#include "visual_slam/core/landmark.h"

namespace visual_slam {

Frame::Ptr Frame::create() {
    static Id factory_id = 0;
    Frame::Ptr new_frame(new Frame);
    new_frame->id = factory_id++;
    return new_frame;
}

void Frame::markAsKeyframe() {
    static Id keyframe_factory_id = 0;
    is_keyframe = true;
    keyframe_id = keyframe_factory_id++;
}

void Frame::clear() {
    is_keyframe = false;
    keyframe_id = 0;
    features_left.clear();
    features_right.clear();
    bow = fbow::fBow();
    bow_keypoints.clear();
    bow_descriptors.release();
    bow_landmarks.clear();
}

void Frame::releaseImagesAndFeatures() {
    image_left.release();
    image_right.release();
    features_left.clear();
    features_right.clear();
}

}  // namespace visual_slam
