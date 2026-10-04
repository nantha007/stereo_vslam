#include "stereo_vslam/backend/local_mapper.h"

#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>

#include "stereo_vslam/backend/optimizer.h"
#include "stereo_vslam/core/feature.h"
#include "stereo_vslam/core/scoped_log_timer.h"

namespace stereo_vslam {

namespace {

constexpr double min_pose_delta = 0.2;

}  // namespace

LocalMapper::LocalMapper(const LocalMapperConfig &config, Map::Ptr map,
                         Camera::Ptr camera_left, Camera::Ptr camera_right)
    : config_(config),
      map_(std::move(map)),
      camera_left_(std::move(camera_left)),
      camera_right_(std::move(camera_right)) {}

bool LocalMapper::insertKeyframe(const std::shared_ptr<const MapState> &base,
                                const Frame::Ptr &keyframe,
                                const TrackResult &result) {
    ScopedLogTimer timer("insertKeyframe");
    if (!base || !keyframe || !map_) 
        return false;

    auto next = std::make_shared<MapState>(*base);
    for (auto &landmark : result.new_landmarks) {
        next->insertLandmark(landmark);
    }
    next->insertKeyframe(keyframe);
    if (result.last_keyframe) {
        PoseGraphEdge edge;
        edge.from_keyframe_id = result.last_keyframe->keyframe_id;
        edge.to_keyframe_id = keyframe->keyframe_id;
        edge.type = EdgeType::Odometry;
        auto from = base->keyframe_poses.find(result.last_keyframe->keyframe_id);
        const SE3 T_from = from != base->keyframe_poses.end()
                               ? from->second
                               : result.last_keyframe->getPose();
        edge.T_from_to = T_from * keyframe->getPose().inverse();
        next->insertEdge(std::move(edge));
    }
    std::optional<Frame::Ptr> dropped = trimActiveWindow(*next, keyframe);
    if (!map_->publish(base->revision, next)) return false;

    registerObservations(keyframe);
    if (dropped) releaseDeactivatedKeyframe(*dropped);
    return true;
}

void LocalMapper::registerObservations(const Frame::Ptr &keyframe) {
    ScopedLogTimer timer("registerObservations");
    for (auto &feature : keyframe->features_left) {
        auto landmark = feature->getLandmark();
        if (landmark) landmark->addObservation(feature);
    }
    for (auto &feature : keyframe->features_right) {
        if (!feature) continue;
        auto landmark = feature->getLandmark();
        if (landmark) landmark->addObservation(feature);
    }
}

std::optional<Frame::Ptr> LocalMapper::trimActiveWindow(
    MapState &draft, const Frame::Ptr &keyframe) {
    ScopedLogTimer timer("trimActiveWindow");
    if (draft.active_keyframes.size() <=
        static_cast<size_t>(config_.max_active_keyframes)) {
        return std::nullopt;
    }

    // Prefer a keyframe that barely moved; otherwise drop the farthest one.
    double farthest_delta = 0, closest_delta = 9999;
    Id farthest_keyframe_id = 0, closest_keyframe_id = 0;
    bool has_candidate = false;
    auto current_pose = draft.keyframe_poses.find(keyframe->keyframe_id);
    if (current_pose == draft.keyframe_poses.end()) return std::nullopt;
    const SE3 T_wc = current_pose->second.inverse();
    for (auto &entry : draft.active_keyframes) {
        if (entry.second == keyframe) continue;
        auto pose = draft.keyframe_poses.find(entry.first);
        if (pose == draft.keyframe_poses.end()) continue;
        const double pose_delta = (pose->second * T_wc).log().norm();
        if (!has_candidate || pose_delta > farthest_delta) {
            farthest_delta = pose_delta;
            farthest_keyframe_id = entry.first;
        }
        if (!has_candidate || pose_delta < closest_delta) {
            closest_delta = pose_delta;
            closest_keyframe_id = entry.first;
        }
        has_candidate = true;
    }
    if (!has_candidate) return std::nullopt;

    const Id id = (closest_delta < min_pose_delta) ? closest_keyframe_id
                                                  : farthest_keyframe_id;
    auto dropped_it = draft.active_keyframes.find(id);
    if (dropped_it == draft.active_keyframes.end()) return std::nullopt;
    Frame::Ptr dropped = dropped_it->second;
    draft.active_keyframes.erase(dropped_it);

    std::unordered_map<Id, int> delta;
    auto accumulate = [&](const Frame::Ptr &frame, int sign) {
        if (!frame) return;
        auto count = [&](const Feature::Ptr &feature) {
            if (!feature) return;
            auto landmark = feature->getLandmark();
            if (landmark) delta[landmark->id] += sign;
        };
        for (auto &feature : frame->features_left) count(feature);
        for (auto &feature : frame->features_right) count(feature);
    };
    // The new keyframe's observations are registered after publish. Count them
    // here so a landmark created on this keyframe is not dropped for having
    // none yet.
    accumulate(keyframe, +1);
    accumulate(dropped, -1);
    for (auto iter = draft.active_landmarks.begin();
         iter != draft.active_landmarks.end();) {
        int extra = 0;
        auto change = delta.find(iter->first);
        if (change != delta.end()) extra = change->second;
        if (iter->second->getNumObservations() + extra <= 0) {
            iter = draft.active_landmarks.erase(iter);
        } else {
            ++iter;
        }
    }
    return dropped;
}

void LocalMapper::releaseDeactivatedKeyframe(const Frame::Ptr &keyframe) {
    if (!keyframe) return;
    LOG(INFO) << "Deactivated keyframe " << keyframe->keyframe_id
              << " from the local window";
    for (auto feature : keyframe->features_left) {
        if (!feature) continue;
        auto landmark = feature->getLandmark();
        if (landmark) landmark->removeObservation(feature);
    }
    for (auto feature : keyframe->features_right) {
        if (!feature) continue;
        auto landmark = feature->getLandmark();
        if (landmark) landmark->removeObservation(feature);
    }
    keyframe->releaseImagesAndFeatures();
}

void LocalMapper::runLocalBA() {
    ScopedLogTimer timer("runLocalBA");
    if (!map_) return;
    auto state = map_->load();
    if (!state || state->active_keyframes.size() < 2) return;

    auto graph = optimizer::buildLocalBAGraph(*state, *camera_left_, *camera_right_);
    if (!graph) return;

    optimizer::LocalBAResult result;
    if (!optimizer::solveLocalBA(*graph, &result)) return;

    auto next = std::make_shared<MapState>(*state);
    std::unordered_set<Id> moved;
    for (auto &pose : result.keyframe_poses) {
        next->keyframe_poses[pose.first] = pose.second;
        moved.insert(pose.first);
    }
    for (auto &position : result.landmark_positions) {
        next->landmark_positions[position.first] = position.second;
    }
    for (auto &edge : next->edges) {
        if (edge.type != EdgeType::Odometry) continue;
        if (!moved.count(edge.from_keyframe_id) &&
            !moved.count(edge.to_keyframe_id)) {
            continue;
        }
        auto from = next->keyframe_poses.find(edge.from_keyframe_id);
        auto to = next->keyframe_poses.find(edge.to_keyframe_id);
        if (from == next->keyframe_poses.end() || to == next->keyframe_poses.end()) {
            continue;
        }
        edge.T_from_to = from->second * to->second.inverse();
    }
    if (!map_->publish(state->revision, std::move(next))) {
        LOG(INFO) << "Discard stale local BA after a global correction";
        return;
    }

    for (auto &feature : result.outlier_features) {
        feature->setOutlier(true);
        if (auto landmark = feature->getLandmark()) {
            landmark->removeObservation(feature);
        }
    }
    for (auto &feature : result.inlier_features) {
        feature->setOutlier(false);
    }

    LOG(INFO) << "Outliers/inliers in local BA: " << result.num_outliers << "/"
              << result.num_inliers;
}

}  // namespace stereo_vslam
