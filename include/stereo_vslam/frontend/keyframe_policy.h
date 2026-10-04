#pragma once

#include <cmath>

#include "stereo_vslam/core/config.h"
#include "stereo_vslam/core/types.h"

namespace stereo_vslam {

/**
 * KeyframePolicy
 * Decides whether a tracked frame becomes a keyframe.
 */
class KeyframePolicy {
   public:
    /** @brief Build a policy from the keyframe thresholds. */
    explicit KeyframePolicy(const KeyframeConfig &config) : config_(config) {}

    /** @brief Rotation between two T_cw poses, in degrees. */
    static double rotationDeg(const SE3 &T_cw_current,
                              const SE3 &T_cw_last_keyframe) {
        const SO3 R_rel =
            T_cw_current.so3() * T_cw_last_keyframe.so3().inverse();
        return R_rel.log().norm() * 180.0 / M_PI;
    }

    /**
     * @brief True when tracking inliers are low or the camera turned enough
     * since the last keyframe.
     */
    bool needsKeyframe(int inliers, const SE3 &T_cw_current,
                       const SE3 &T_cw_last_keyframe) const {
        return inliers < config_.min_inliers ||
               rotationDeg(T_cw_current, T_cw_last_keyframe) >=
                   config_.max_rotation_deg;
    }

   private:
    KeyframeConfig config_;
};

}  // namespace stereo_vslam
