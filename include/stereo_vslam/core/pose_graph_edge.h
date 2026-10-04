#pragma once

#include "stereo_vslam/core/types.h"

namespace stereo_vslam {

enum class EdgeType { Odometry, LoopClosure };

/// Pose-pose constraint. Measurement is T_from_to = T_from * T_to.inverse(),
/// with both poses in T_cw, matching EdgeRelativePose.
struct PoseGraphEdge {

    Id from_keyframe_id = 0;
    Id to_keyframe_id = 0;
    SE3 T_from_to;
    EdgeType type = EdgeType::Odometry;
    Mat66 information = 100.0 * Mat66::Identity();
};

}  // namespace stereo_vslam
