#pragma once

#include <memory>

#include "stereo_vslam/core/types.h"

namespace stereo_vslam {

/** @brief Pinhole stereo camera model. */
class Camera {
   public:
    using Ptr = std::shared_ptr<Camera>;

    double fx = 0, fy = 0, cx = 0, cy = 0, baseline = 0;  // Camera intrinsics

    /**
     * @brief Build a camera from intrinsics, baseline, and the stereo extrinsic.
     * @param extrinsic Transform from the stereo camera to this camera.
     */
    Camera(double fx, double fy, double cx, double cy, double baseline,
           const SE3 &extrinsic)
        : fx(fx),
          fy(fy),
          cx(cx),
          cy(cy),
          baseline(baseline),
          extrinsic_(extrinsic) {}

    /** @brief Extrinsic from the stereo camera to this camera. */
    SE3 getExtrinsic() const { return extrinsic_; }

    /** @brief Intrinsic matrix. */
    Mat33 getIntrinsicMatrix() const {
        Mat33 K;
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        return K;
    }

    /**
     * @brief Transform a world point into this camera.
     * @param T_cw Pose of the stereo camera, in T_cw.
     */
    Vec3 worldToCamera(const Vec3 &p_w, const SE3 &T_cw);

    /** @brief Project a point in this camera frame to pixels. */
    Vec2 cameraToPixel(const Vec3 &p_c);

    /**
     * @brief Project a world point to pixels.
     * @param T_cw Pose of the stereo camera, in T_cw.
     */
    Vec2 worldToPixel(const Vec3 &p_w, const SE3 &T_cw);

   private:
    SE3 extrinsic_;  // from stereo camera to this camera
};

}  // namespace stereo_vslam
