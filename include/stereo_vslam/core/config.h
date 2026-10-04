#pragma once

#include <string>

#include "stereo_vslam/core/types.h"

namespace stereo_vslam {

struct DatasetConfig {
    std::string dataset_path;
};

/// YAML section `feature_extractor` (ORB detector).
struct FeatureExtractorConfig {
    int num_features;
};

struct KeyframeConfig {
    double max_rotation_deg;
    int min_inliers;
};

struct FarLandmarkFilterConfig {
    bool enabled;
    double max_distance_m;

    /**
     * @brief True when far-point filtering is on and @p p_c is beyond the limit.
     * @param p_c Point in the left camera frame.
     */
    bool isFar(const Vec3 &p_c) const {
        return enabled && p_c.norm() > max_distance_m;
    }
};

struct TrackingConfig {
    int min_pose_inliers;
    KeyframeConfig keyframe;
    FarLandmarkFilterConfig far_landmark_filter;
};

struct LocalMapperConfig {
    int max_active_keyframes;
};

struct LoopClosureConfig {
    int exclude_recent_keyframes;
    int cooldown_keyframes;
    double search_radius_m;
    double min_bow_score;
    int max_bow_candidates;
    int min_pnp_inliers;
    std::string vocabulary_path;
};

/// YAML section `viewer`. Missing section keeps the window on.
struct ViewerConfig {
    bool enabled = true;
};

struct SlamConfig {
    DatasetConfig dataset;
    FeatureExtractorConfig feature_extractor;
    TrackingConfig tracking;
    LocalMapperConfig local_mapper;
    LoopClosureConfig loop_closure;
    ViewerConfig viewer;

    /** @brief Load every section from a YAML file. */
    explicit SlamConfig(const std::string &filename);
};

}  // namespace stereo_vslam
