#include "stereo_vslam/core/camera.h"

namespace stereo_vslam {

Vec3 Camera::worldToCamera(const Vec3 &p_w, const SE3 &T_cw) {
    return extrinsic_ * T_cw * p_w;
}

Vec2 Camera::cameraToPixel(const Vec3 &p_c) {
    return Vec2(
            fx * p_c(0, 0) / p_c(2, 0) + cx,
            fy * p_c(1, 0) / p_c(2, 0) + cy
    );
}

Vec2 Camera::worldToPixel(const Vec3 &p_w, const SE3 &T_cw) {
    return cameraToPixel(worldToCamera(p_w, T_cw));
}

}  // namespace stereo_vslam
