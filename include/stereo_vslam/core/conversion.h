#pragma once

#include <opencv2/core.hpp>

#include "stereo_vslam/core/types.h"

namespace stereo_vslam {

/** @brief Convert a pixel to a 2-vector. */
inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

/** @brief Convert a 2-vector to a pixel. */
inline cv::Point2f toPoint2f(const Vec2 &v) {
    return cv::Point2f(static_cast<float>(v[0]), static_cast<float>(v[1]));
}

/** @brief Convert a 3-vector to a float point. */
inline cv::Point3f toPoint3f(const Vec3 &v) {
    return cv::Point3f(static_cast<float>(v[0]), static_cast<float>(v[1]),
                       static_cast<float>(v[2]));
}

/** @brief Copy a 3x3 matrix into a CV_64F Mat. */
inline cv::Mat toCvMat(const Mat33 &m) {
    cv::Mat out(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out.at<double>(i, j) = m(i, j);
        }
    }
    return out;
}

/** @brief Copy a 3x3 CV_64F Mat into a matrix. */
inline Mat33 toMat33(const cv::Mat &m) {
    Mat33 out;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out(i, j) = m.at<double>(i, j);
        }
    }
    return out;
}

/** @brief Copy a 3x1 CV_64F Mat into a vector. */
inline Vec3 toVec3(const cv::Mat &v) {
    return Vec3(v.at<double>(0), v.at<double>(1), v.at<double>(2));
}

}  // namespace stereo_vslam
