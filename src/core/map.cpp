#include "stereo_vslam/core/map.h"

namespace stereo_vslam {

void MapState::insertKeyframe(Frame::Ptr frame) {
    keyframes[frame->keyframe_id] = frame;
    active_keyframes[frame->keyframe_id] = frame;
    keyframe_poses[frame->keyframe_id] = frame->getPose();
}

void MapState::insertLandmark(Landmark::Ptr landmark) {
    landmarks[landmark->id] = landmark;
    active_landmarks[landmark->id] = landmark;
    landmark_positions[landmark->id] = landmark->getPosition();
}

void MapState::insertEdge(PoseGraphEdge edge) { edges.push_back(std::move(edge)); }

MapState::KeyframeIdPairs MapState::loopKeyframePairs() const {
    KeyframeIdPairs pairs;
    for (const auto &edge : edges) {
        if (edge.type != EdgeType::LoopClosure) continue;
        Id a = edge.from_keyframe_id;
        Id b = edge.to_keyframe_id;
        if (a > b) std::swap(a, b);
        pairs.emplace_back(a, b);
    }
    return pairs;
}

Map::Map() : state_(std::make_shared<MapState>()) {}

std::shared_ptr<const MapState> Map::load() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

bool Map::publish(uint64_t base_revision, std::shared_ptr<MapState> next) {
    if (!next) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_ || state_->revision != base_revision) return false;
    next->revision = base_revision + 1;
    state_ = std::move(next);
    return true;
}

}  // namespace stereo_vslam
