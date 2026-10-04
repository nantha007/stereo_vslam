#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "stereo_vslam/core/camera.h"
#include "stereo_vslam/core/feature.h"
#include "stereo_vslam/core/frame.h"
#include "stereo_vslam/core/map.h"
#include "stereo_vslam/core/types.h"

namespace stereo_vslam {
namespace optimizer {

/// Solved local window. Poses and positions are not written back.
struct LocalBAResult {
    PoseMap keyframe_poses;
    std::unordered_map<Id, Vec3> landmark_positions;
    std::vector<Feature::Ptr> outlier_features;
    std::vector<Feature::Ptr> inlier_features;
    int num_outliers = 0;
    int num_inliers = 0;
};

/// Pose-graph solution. poses_before are the estimates the solver started from.
struct PoseGraphResult {
    PoseMap poses_before;
    PoseMap poses_after;
    Id newest_keyframe_id = 0;
    bool is_valid = false;
};

/**
 * @brief Refine @p pose by pose-only bundle adjustment and mark outliers on
 * @p frame.
 *
 * Landmark positions come from @p state. A landmark missing there uses its
 * private position.
 * @param pose Pose to refine, in T_cw.
 * @return Inlier count.
 */
int optimizePose(Frame &frame, const Camera &camera_left, const MapState &state,
                 SE3 *pose);

/// Reprojection graph built from the snapshot's active window.
/// Observations are read from the landmark objects. Empty when the window has
/// fewer than two keyframes or no edges.
class LocalBAGraph {
   public:
    /** @brief Move-construct a graph. */
    LocalBAGraph(LocalBAGraph &&) noexcept;

    /** @brief Move-assign a graph. */
    LocalBAGraph &operator=(LocalBAGraph &&) noexcept;

    /** @brief Destroy the graph. */
    ~LocalBAGraph();

    /** @brief Copying a graph is not supported. */
    LocalBAGraph(const LocalBAGraph &) = delete;

    /** @brief Copying a graph is not supported. */
    LocalBAGraph &operator=(const LocalBAGraph &) = delete;

   private:
    struct Impl;

    /** @brief Take ownership of a built graph. */
    explicit LocalBAGraph(std::unique_ptr<Impl> impl);

    std::unique_ptr<Impl> impl_;

    friend std::unique_ptr<LocalBAGraph> buildLocalBAGraph(
        const MapState &state, const Camera &camera_left,
        const Camera &camera_right);
    friend bool solveLocalBA(LocalBAGraph &graph, LocalBAResult *result);
};

/**
 * @brief Copy the active window into a g2o reprojection graph.
 *
 * All but the two newest keyframes are held fixed. With exactly two
 * keyframes, the older one is held fixed.
 * @return Empty when the window has fewer than two keyframes or no edges.
 */
std::unique_ptr<LocalBAGraph> buildLocalBAGraph(
    const MapState &state, const Camera &camera_left,
    const Camera &camera_right);

/**
 * @brief Solve a graph already built by buildLocalBAGraph().
 *
 * Does not read live poses or landmark positions.
 * @return False when @p result is null.
 */
bool solveLocalBA(LocalBAGraph &graph, LocalBAResult *result);

/**
 * @brief Optimize the pose graph. Keyframe 0 is fixed.
 *
 * Every edge uses its stored T_from_to. Live keyframe poses are not read.
 */
PoseGraphResult optimizePoseGraph(const PoseMap &poses,
                                  const Map::EdgeList &edges);

}  // namespace optimizer
}  // namespace stereo_vslam
