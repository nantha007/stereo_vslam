#pragma once

#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "stereo_vslam/core/frame.h"
#include "stereo_vslam/core/landmark.h"
#include "stereo_vslam/core/pose_graph_edge.h"
#include "stereo_vslam/core/types.h"

namespace stereo_vslam {

/**
 * Published map.
 * Immutable after Map::publish. Poses and landmark positions in this struct
 * are the geometry readers use. Frame and Landmark objects are shared for
 * images, features, BoW, and observations; those fields are written on the
 * main thread only. The loop thread reads BoW and this snapshot.
 */
struct MapState {
    using LandmarkMap = std::unordered_map<Id, Landmark::Ptr>;
    using KeyframeMap = std::unordered_map<Id, Frame::Ptr>;
    using EdgeList = std::vector<PoseGraphEdge,
                                 Eigen::aligned_allocator<PoseGraphEdge>>;
    using KeyframeIdPairs = std::vector<std::pair<Id, Id>>;
    using LandmarkPositionMap = std::unordered_map<Id, Vec3>;

    /// Increments only when a loop correction commits.
    uint64_t epoch = 0;
    /// Increments on every successful publish.
    uint64_t revision = 0;
    /// Product of every loop correction so far. Identity before the first.
    SE3 accumulated_correction;

    LandmarkMap landmarks;
    LandmarkMap active_landmarks;
    KeyframeMap keyframes;
    KeyframeMap active_keyframes;
    PoseMap keyframe_poses;
    LandmarkPositionMap landmark_positions;
    EdgeList edges;

    /** @brief Insert @p frame into the keyframe maps and store its pose. */
    void insertKeyframe(Frame::Ptr frame);

    /** @brief Insert @p landmark and store its current position. */
    void insertLandmark(Landmark::Ptr landmark);

    /** @brief Append a pose-graph edge. */
    void insertEdge(PoseGraphEdge edge);

    /**
     * @brief Loop-closure keyframe id pairs, ordered so the smaller id is first.
     */
    KeyframeIdPairs loopKeyframePairs() const;
};

/**
 * @brief Snapshot position when the landmark is published, otherwise its
 * private one.
 */
inline Vec3 landmarkPosition(const MapState &state, const Landmark &landmark) {
    auto it = state.landmark_positions.find(landmark.id);
    if (it != state.landmark_positions.end()) return it->second;
    return landmark.getPosition();
}

/**
 * Map
 * Publishes an immutable MapState. load() copies the pointer. publish()
 * swaps it under a mutex and fails when base_revision is no longer current.
 * Callers fill the next state without holding the mutex.
 */
class Map {
   public:
    using Ptr = std::shared_ptr<Map>;
    using LandmarkMap = MapState::LandmarkMap;
    using KeyframeMap = MapState::KeyframeMap;
    using EdgeList = MapState::EdgeList;
    using KeyframeIdPairs = MapState::KeyframeIdPairs;

    /** @brief Create an empty map. */
    Map();

    /** @brief Current published snapshot. */
    std::shared_ptr<const MapState> load() const;

    /**
     * @brief Publish @p next if the current revision is still @p base_revision.
     *
     * On success, next->revision becomes @p base_revision + 1.
     * @return False when @p next is null or the revision has already moved.
     */
    bool publish(uint64_t base_revision, std::shared_ptr<MapState> next);

   private:
    mutable std::mutex mutex_;
    std::shared_ptr<const MapState> state_;
};

}  // namespace stereo_vslam
