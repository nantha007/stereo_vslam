#pragma once

#include <cstdint>
#include <unordered_map>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace stereo_vslam {

using Mat66 = Eigen::Matrix<double, 6, 6>;
using Mat33 = Eigen::Matrix<double, 3, 3>;
using Mat22 = Eigen::Matrix<double, 2, 2>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec2 = Eigen::Matrix<double, 2, 1>;

using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;

using Id = uint64_t;
using PoseMap = std::unordered_map<Id, SE3>;

}  // namespace stereo_vslam
