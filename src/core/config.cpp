#include "visual_slam/core/config.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace visual_slam {
namespace config_detail {

bool tryGetScalar(const YAML::Node &parent, const char *key, YAML::Node *out) {
    if (!parent.IsMap() || !parent[key]) {
        return false;
    }
    // Copy construction shares the node. operator= calls set_ref and would
    // retarget the previous key in this document.
    const YAML::Node child = parent[key];
    if (!child.IsScalar()) {
        return false;
    }
    out->reset(child);
    return true;
}

bool tryGetMap(const YAML::Node &parent, const char *key, YAML::Node *out) {
    if (!parent.IsMap() || !parent[key]) {
        return false;
    }
    const YAML::Node child = parent[key];
    if (!child.IsMap()) {
        return false;
    }
    out->reset(child);
    return true;
}

}  // namespace config_detail
}  // namespace visual_slam

namespace YAML {

template <>
struct convert<visual_slam::DatasetConfig> {
    static Node encode(const visual_slam::DatasetConfig &rhs) {
        Node node;
        node["dataset_path"] = rhs.dataset_path;
        return node;
    }

    static bool decode(const Node &node, visual_slam::DatasetConfig &rhs) {
        YAML::Node scalar;
        if (!visual_slam::config_detail::tryGetScalar(node, "dataset_path",
                                                        &scalar)) {
            return false;
        }
        rhs.dataset_path = scalar.as<std::string>();
        return true;
    }
};

template <>
struct convert<visual_slam::FeatureExtractorConfig> {
    static bool decode(const Node &node, visual_slam::FeatureExtractorConfig &rhs) {
        YAML::Node scalar;
        if (!visual_slam::config_detail::tryGetScalar(node, "num_features",
                                                        &scalar)) {
            return false;
        }
        rhs.num_features = scalar.as<int>();
        rhs.type = visual_slam::FeatureType::Orb;
        if (node["type"]) {
            if (!visual_slam::config_detail::tryGetScalar(node, "type",
                                                            &scalar)) {
                return false;
            }
            const std::string type = scalar.as<std::string>();
            if (type == "orb") {
                rhs.type = visual_slam::FeatureType::Orb;
            } else if (type == "superpoint") {
                rhs.type = visual_slam::FeatureType::SuperPoint;
            } else {
                return false;
            }
        }
        if (rhs.type != visual_slam::FeatureType::SuperPoint) {
            return true;
        }
        if (!visual_slam::config_detail::tryGetScalar(node, "superpoint_model",
                                                        &scalar) ||
            !visual_slam::config_detail::tryGetScalar(node, "lightglue_model",
                                                        &scalar)) {
            return false;
        }
        rhs.superpoint_model = node["superpoint_model"].as<std::string>();
        rhs.lightglue_model = node["lightglue_model"].as<std::string>();
        return true;
    }
};

template <>
struct convert<visual_slam::KeyframeConfig> {
    static bool decode(const Node &node, visual_slam::KeyframeConfig &rhs) {
        YAML::Node scalar;
        if (!visual_slam::config_detail::tryGetScalar(node, "max_rotation_deg",
                                                        &scalar)) {
            return false;
        }
        rhs.max_rotation_deg = scalar.as<double>();
        if (!visual_slam::config_detail::tryGetScalar(node, "min_inliers",
                                                        &scalar)) {
            return false;
        }
        rhs.min_inliers = scalar.as<int>();
        return true;
    }
};

template <>
struct convert<visual_slam::FarLandmarkFilterConfig> {
    static bool decode(const Node &node,
                       visual_slam::FarLandmarkFilterConfig &rhs) {
        YAML::Node enabled_node;
        YAML::Node max_distance_m_node;
        if (!visual_slam::config_detail::tryGetScalar(node, "enabled",
                                                        &enabled_node) ||
            !visual_slam::config_detail::tryGetScalar(node, "max_distance_m",
                                                        &max_distance_m_node)) {
            return false;
        }
        rhs.enabled = enabled_node.as<bool>();
        rhs.max_distance_m = max_distance_m_node.as<double>();
        return true;
    }
};

template <>
struct convert<visual_slam::TrackingConfig> {
    static bool decode(const Node &node, visual_slam::TrackingConfig &rhs) {
        YAML::Node scalar;
        YAML::Node keyframe_node;
        YAML::Node far_node;
        if (!visual_slam::config_detail::tryGetScalar(node, "min_pose_inliers",
                                                        &scalar) ||
            !visual_slam::config_detail::tryGetMap(node, "keyframe",
                                                     &keyframe_node) ||
            !visual_slam::config_detail::tryGetMap(node, "far_landmark_filter",
                                                     &far_node)) {
            return false;
        }
        rhs.min_pose_inliers = node["min_pose_inliers"].as<int>();
        if (!convert<visual_slam::KeyframeConfig>::decode(keyframe_node,
                                                           rhs.keyframe)) {
            return false;
        }
        if (!convert<visual_slam::FarLandmarkFilterConfig>::decode(
                far_node, rhs.far_landmark_filter)) {
            return false;
        }
        return true;
    }
};

template <>
struct convert<visual_slam::LocalMapperConfig> {
    static bool decode(const Node &node, visual_slam::LocalMapperConfig &rhs) {
        YAML::Node scalar;
        if (!visual_slam::config_detail::tryGetScalar(
                node, "max_active_keyframes", &scalar)) {
            return false;
        }
        rhs.max_active_keyframes = scalar.as<int>();
        return true;
    }
};

template <>
struct convert<visual_slam::LoopClosureConfig> {
    static bool decode(const Node &node, visual_slam::LoopClosureConfig &rhs) {
        YAML::Node scalar;
        if (!visual_slam::config_detail::tryGetScalar(
                node, "exclude_recent_keyframes", &scalar) ||
            !visual_slam::config_detail::tryGetScalar(
                node, "cooldown_keyframes", &scalar) ||
            !visual_slam::config_detail::tryGetScalar(node, "search_radius_m",
                                                        &scalar) ||
            !visual_slam::config_detail::tryGetScalar(node, "min_bow_score",
                                                        &scalar) ||
            !visual_slam::config_detail::tryGetScalar(
                node, "max_bow_candidates", &scalar) ||
            !visual_slam::config_detail::tryGetScalar(node, "min_pnp_inliers",
                                                        &scalar) ||
            !visual_slam::config_detail::tryGetScalar(node, "vocabulary_path",
                                                        &scalar)) {
            return false;
        }
        rhs.exclude_recent_keyframes = node["exclude_recent_keyframes"].as<int>();
        rhs.cooldown_keyframes = node["cooldown_keyframes"].as<int>();
        rhs.search_radius_m = node["search_radius_m"].as<double>();
        rhs.min_bow_score = node["min_bow_score"].as<double>();
        rhs.max_bow_candidates = node["max_bow_candidates"].as<int>();
        rhs.min_pnp_inliers = node["min_pnp_inliers"].as<int>();
        rhs.vocabulary_path = node["vocabulary_path"].as<std::string>();
        return true;
    }
};

template <>
struct convert<visual_slam::ViewerConfig> {
    static bool decode(const Node &node, visual_slam::ViewerConfig &rhs) {
        YAML::Node scalar;
        if (!visual_slam::config_detail::tryGetScalar(node, "enabled",
                                                        &scalar)) {
            return false;
        }
        rhs.enabled = scalar.as<bool>();
        return true;
    }
};

template <>
struct convert<visual_slam::SlamConfig> {
    static bool decode(const Node &node, visual_slam::SlamConfig &rhs) {
        YAML::Node dataset_node;
        YAML::Node feature_extractor_node;
        YAML::Node tracking_node;
        YAML::Node local_mapper_node;
        YAML::Node loop_closure_node;
        if (!visual_slam::config_detail::tryGetMap(node, "dataset",
                                                     &dataset_node) ||
            !visual_slam::config_detail::tryGetMap(node, "feature_extractor",
                                                     &feature_extractor_node) ||
            !visual_slam::config_detail::tryGetMap(node, "tracking",
                                                     &tracking_node) ||
            !visual_slam::config_detail::tryGetMap(node, "local_mapper",
                                                     &local_mapper_node) ||
            !visual_slam::config_detail::tryGetMap(node, "loop_closure",
                                                     &loop_closure_node)) {
            return false;
        }
        if (!convert<visual_slam::DatasetConfig>::decode(dataset_node,
                                                          rhs.dataset)) {
            return false;
        }
        if (!convert<visual_slam::FeatureExtractorConfig>::decode(
                feature_extractor_node, rhs.feature_extractor)) {
            return false;
        }
        if (!convert<visual_slam::TrackingConfig>::decode(tracking_node,
                                                          rhs.tracking)) {
            return false;
        }
        if (!convert<visual_slam::LocalMapperConfig>::decode(
                local_mapper_node, rhs.local_mapper)) {
            return false;
        }
        if (!convert<visual_slam::LoopClosureConfig>::decode(
                loop_closure_node, rhs.loop_closure)) {
            return false;
        }
        if (node["viewer"]) {
            YAML::Node viewer_node;
            if (!visual_slam::config_detail::tryGetMap(node, "viewer",
                                                         &viewer_node) ||
                !convert<visual_slam::ViewerConfig>::decode(viewer_node,
                                                             rhs.viewer)) {
                return false;
            }
        }
        return true;
    }
};

}  // namespace YAML

namespace visual_slam {

SlamConfig::SlamConfig(const std::string &filename) {
    try {
        const YAML::Node root = YAML::LoadFile(filename);
        CHECK(root.IsMap()) << "config file " << filename << " is not a YAML map.";
        CHECK(YAML::convert<SlamConfig>::decode(root, *this))
            << "config file " << filename
            << " is missing required keys or has invalid values.";
    } catch (const YAML::Exception &e) {
        LOG(FATAL) << "failed to load config file " << filename << ": "
                   << e.what();
    }
}

}  // namespace visual_slam
