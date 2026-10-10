#include "stereo_vslam/loop/loop_closure.h"

#include <algorithm>
#include <unordered_set>

#include <glog/logging.h>
#include <nanoflann.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

#include "stereo_vslam/backend/optimizer.h"
#include "stereo_vslam/core/conversion.h"
#include "stereo_vslam/core/landmark.h"
#include "stereo_vslam/core/scoped_log_timer.h"
#include "stereo_vslam/frontend/light_glue.h"


namespace stereo_vslam {

namespace {

struct KeyframeCloud {
    std::vector<Vec3> points;

    inline size_t kdtree_get_point_count() const { return points.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return points[idx][static_cast<int>(dim)];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX &) const {
        return false;
    }
};

using KeyframeKDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, KeyframeCloud>, KeyframeCloud, 3>;

Features featuresFromBow(const Frame &frame) {
    const cv::Mat descriptors = frame.bow_descriptors.isContinuous()
                                    ? frame.bow_descriptors
                                    : frame.bow_descriptors.clone();
    const int count = std::min(descriptors.rows,
                               static_cast<int>(frame.bow_keypoints.size()));
    Features features;
    features.descriptor_dim = descriptors.cols;
    features.keypoints.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        features.keypoints.push_back(frame.bow_keypoints[i].pt);
    }
    const size_t n =
        static_cast<size_t>(count) * static_cast<size_t>(descriptors.cols);
    features.descriptors.resize(n);
    if (n > 0) {
        std::copy(descriptors.ptr<float>(), descriptors.ptr<float>() + n,
                  features.descriptors.begin());
    }
    return features;
}

}  // namespace

LoopClosure::LoopClosure(const LoopClosureConfig &config, Map::Ptr map,
                         Camera::Ptr camera_left,
                         LoopClosedCallback on_loop_closed,
                         LightGlue::Ptr light_glue)
    : config_(config),
      map_(std::move(map)),
      camera_left_(std::move(camera_left)),
      on_loop_closed_(std::move(on_loop_closed)),
      light_glue_(std::move(light_glue)) {}

void LoopClosure::start() {
    if (loop_thread_.joinable()) return;
    loop_thread_ = std::jthread([this](std::stop_token token) {
        runLoop(std::move(token));
    });
}

LoopClosure::~LoopClosure() { stop(); }

void LoopClosure::submitKeyframe(Frame::Ptr keyframe) {
    ScopedLogTimer timer("submitKeyframe");
    std::unique_lock<std::mutex> lock(pending_mutex_);
    pending_keyframe_ = keyframe;
    has_new_keyframe_ = true;
    keyframe_cv_.notify_one();
}

void LoopClosure::stop() {
    loop_thread_.request_stop();
    if (loop_thread_.joinable()) {
        loop_thread_.join();
    }
}

void LoopClosure::runLoop(std::stop_token token) {
    while (!token.stop_requested()) {
        Frame::Ptr query;
        {
            std::unique_lock<std::mutex> lock(pending_mutex_);
            keyframe_cv_.wait(lock, token, [&] {
                return has_new_keyframe_ || token.stop_requested();
            });
            if (token.stop_requested()) break;
            query = pending_keyframe_;
            has_new_keyframe_ = false;
        }
        if (query) detect(query);
    }
}

void LoopClosure::detect(Frame::Ptr query) {
    if (!map_ || !query) return;
    if (last_loop_keyframe_id_ > 0 &&
        query->keyframe_id < last_loop_keyframe_id_ +
                                 static_cast<Id>(config_.cooldown_keyframes)) {
        return;
    }
    if (query->bow.empty() || query->bow_descriptors.empty()) return;

    auto state = map_->load();
    if (!state) return;
    std::vector<Frame::Ptr> candidates =
        spatialCandidates(query, state->keyframes, state->keyframe_poses);
    LOG(INFO) << "Loop closure: " << candidates.size()
              << " keyframes within " << config_.search_radius_m
              << " m and not in the last " << config_.exclude_recent_keyframes
              << " of keyframe " << query->keyframe_id;
    if (candidates.empty()) return;

    std::vector<std::pair<double, Frame::Ptr>> scored;
    scored.reserve(candidates.size());
    for (auto &candidate : candidates) {
        if (candidate->bow.empty()) continue;
        const double score = fbow::fBow::score(query->bow, candidate->bow);
        scored.emplace_back(score, candidate);
    }
    std::sort(scored.begin(), scored.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });

    int verified = 0;
    for (auto &item : scored) {
        if (item.first < config_.min_bow_score) break;
        if (verified >= config_.max_bow_candidates) break;
        auto candidate = item.second;
        std::pair<Id, Id> ids{
            std::min(query->keyframe_id, candidate->keyframe_id),
            std::max(query->keyframe_id, candidate->keyframe_id)};
        if (closed_pairs_.count(ids)) continue;

        ++verified;
        LOG(INFO) << "Loop candidate keyframe " << candidate->keyframe_id
                  << " with FBoW score " << item.first;

        SE3 T_cw_query;
        if (!estimateLoopPose(query, candidate, *state, &T_cw_query)) {
            LOG(INFO) << "Geometric verification failed for keyframe "
                      << candidate->keyframe_id;
            continue;
        }

        auto pose_it = state->keyframe_poses.find(candidate->keyframe_id);
        if (pose_it == state->keyframe_poses.end()) continue;

        PoseGraphEdge edge;
        edge.from_keyframe_id = query->keyframe_id;
        edge.to_keyframe_id = candidate->keyframe_id;
        edge.T_from_to = T_cw_query * pose_it->second.inverse();
        edge.type = EdgeType::LoopClosure;
        LOG(INFO) << "Loop closure accepted between keyframes "
                  << query->keyframe_id << " and " << candidate->keyframe_id
                  << " (FBoW score " << item.first << ")";
        if (!commitLoop(edge)) break;
        closed_pairs_.insert(ids);
        last_loop_keyframe_id_ = query->keyframe_id;
        break;
    }
}

bool LoopClosure::commitLoop(const PoseGraphEdge &edge) {
    bool saw_stale = false;
    for (int attempt = 0; attempt < 2; ++attempt) {
        bool stale = false;
        if (closeLoop(edge, &stale)) return true;
        if (!stale) return false;
        saw_stale = true;
    }
    if (saw_stale) {
        LOG(INFO) << "Dropped loop correction after a revision race";
    }
    return false;
}

bool LoopClosure::closeLoop(const PoseGraphEdge &edge, bool *stale) {
    if (stale) *stale = false;
    if (!map_) return false;
    auto state = map_->load();
    if (!state) return false;

    Map::EdgeList edges = state->edges;
    edges.push_back(edge);
    optimizer::PoseGraphResult solved =
        optimizer::optimizePoseGraph(state->keyframe_poses, edges);
    if (!solved.is_valid) return false;

    std::unordered_set<Id> optimized_ids;
    for (auto &kv : solved.poses_after) optimized_ids.insert(kv.first);

    const SE3 T_cw_newest_before =
        solved.poses_before.at(solved.newest_keyframe_id);

    auto next = std::make_shared<MapState>(*state);
    next->edges.push_back(edge);
    for (auto &kv : solved.poses_after) {
        next->keyframe_poses[kv.first] = kv.second;
    }
    const SE3 T_cw_newest_after =
        next->keyframe_poses.at(solved.newest_keyframe_id);
    const SE3 T_correction = T_cw_newest_before.inverse() * T_cw_newest_after;

    int num_propagated_keyframes = 0;
    for (auto &kv : next->keyframe_poses) {
        if (optimized_ids.count(kv.first)) continue;
        kv.second = kv.second * T_correction;
        num_propagated_keyframes++;
    }

    std::unordered_set<Id> updated;
    auto position_of = [&](const Landmark::Ptr &landmark) {
        auto it = next->landmark_positions.find(landmark->id);
        if (it != next->landmark_positions.end()) return it->second;
        return landmark->getPosition();
    };
    for (auto &kv : next->landmarks) {
        auto landmark = kv.second;
        if (!landmark || !landmark->hasRefKeyframe()) continue;
        if (updated.count(landmark->id)) continue;
        const Id ref_id = landmark->getRefKeyframeId();
        const Vec3 position = position_of(landmark);
        if (optimized_ids.count(ref_id)) {
            auto before = solved.poses_before.find(ref_id);
            auto after = next->keyframe_poses.find(ref_id);
            if (before == solved.poses_before.end() ||
                after == next->keyframe_poses.end()) {
                continue;
            }
            const Vec3 p_c = before->second * position;
            next->landmark_positions[landmark->id] = after->second.inverse() * p_c;
        } else {
            const Vec3 p_c = T_cw_newest_before * position;
            next->landmark_positions[landmark->id] =
                T_cw_newest_after.inverse() * p_c;
        }
        updated.insert(landmark->id);
    }

    next->accumulated_correction = state->accumulated_correction * T_correction;
    next->epoch = state->epoch + 1;
    PoseMap poses_before = state->keyframe_poses;
    PoseMap poses_after = next->keyframe_poses;
    if (!map_->publish(state->revision, std::move(next))) {
        if (stale) *stale = true;
        return false;
    }

    LOG(INFO) << "Pose graph optimization updated " << updated.size()
              << " landmarks, propagated keyframes " << num_propagated_keyframes
              << ", correction translation " << T_correction.translation().norm()
              << " m";
    if (on_loop_closed_) on_loop_closed_(poses_before, poses_after);
    return true;
}

std::vector<Frame::Ptr> LoopClosure::spatialCandidates(
    const Frame::Ptr &query, const Map::KeyframeMap &keyframes,
    const PoseMap &poses) {
    std::vector<Frame::Ptr> cloud_keyframes;
    KeyframeCloud cloud;
    cloud.points.reserve(keyframes.size());
    cloud_keyframes.reserve(keyframes.size());

    for (auto &kv : keyframes) {
        auto keyframe = kv.second;
        if (!keyframe || keyframe == query) continue;
        if (static_cast<int>(keyframe->keyframe_id) +
                config_.exclude_recent_keyframes >
            static_cast<int>(query->keyframe_id)) {
            continue;
        }
        auto pose_it = poses.find(keyframe->keyframe_id);
        if (pose_it == poses.end()) continue;
        cloud.points.push_back(pose_it->second.inverse().translation());
        cloud_keyframes.push_back(keyframe);
    }

    std::vector<Frame::Ptr> candidates;
    if (cloud.points.empty()) return candidates;

    auto query_pose = poses.find(query->keyframe_id);
    if (query_pose == poses.end()) return candidates;

    KeyframeKDTree index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    const Vec3 q = query_pose->second.inverse().translation();
    const double query_pt[3] = {q[0], q[1], q[2]};
    const double radius2 =
        config_.search_radius_m * config_.search_radius_m;
    std::vector<nanoflann::ResultItem<uint32_t, double>> matches;
    index.radiusSearch(query_pt, radius2, matches);

    candidates.reserve(matches.size());
    for (auto &m : matches) {
        candidates.push_back(cloud_keyframes[m.first]);
    }
    return candidates;
}

bool LoopClosure::estimateLoopPose(const Frame::Ptr &query,
                                   const Frame::Ptr &match,
                                   const MapState &state, SE3 *T_cw_query) {
    if (!camera_left_ || !T_cw_query) return false;
    if (query->bow_descriptors.empty() || match->bow_descriptors.empty()) {
        return false;
    }

    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    if (light_glue_) {
        if (query->bow_descriptors.type() != CV_32F ||
            match->bow_descriptors.type() != CV_32F) {
            LOG(WARNING) << "LightGlue expects CV_32F descriptors";
            return false;
        }
        const Features features_query = featuresFromBow(*query);
        const Features features_match = featuresFromBow(*match);
        const std::vector<Match> matches = light_glue_->match(
            features_query, query->image_size, features_match, match->image_size);
        for (const auto &m : matches) {
            if (m.index0 < 0 || m.index1 < 0) continue;
            if (m.index0 >= static_cast<int>(query->bow_keypoints.size()) ||
                m.index1 >= static_cast<int>(match->bow_landmarks.size())) {
                continue;
            }
            auto landmark = match->bow_landmarks[m.index1].lock();
            if (!landmark) continue;
            auto position = state.landmark_positions.find(landmark->id);
            if (position == state.landmark_positions.end()) continue;
            obj.push_back(toPoint3f(position->second));
            img.push_back(query->bow_keypoints[m.index0].pt);
        }
    } else {
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(query->bow_descriptors, match->bow_descriptors, knn, 2);

        const double ratio = 0.75;
        for (auto &m : knn) {
            if (m.size() < 2) continue;
            if (m[0].distance > ratio * m[1].distance) continue;
            const int train_idx = m[0].trainIdx;
            const int query_idx = m[0].queryIdx;
            if (train_idx < 0 ||
                query_idx < 0 ||
                train_idx >= static_cast<int>(match->bow_landmarks.size()) ||
                query_idx >= static_cast<int>(query->bow_keypoints.size())) {
                continue;
            }
            auto landmark = match->bow_landmarks[train_idx].lock();
            if (!landmark) continue;
            auto position = state.landmark_positions.find(landmark->id);
            if (position == state.landmark_positions.end()) continue;
            obj.push_back(toPoint3f(position->second));
            img.push_back(query->bow_keypoints[query_idx].pt);
        }
    }

    if (static_cast<int>(obj.size()) < config_.min_pnp_inliers) {
        LOG(INFO) << "Not enough 3D-2D matches for PnP: " << obj.size();
        return false;
    }

    cv::Mat K_cv = toCvMat(camera_left_->getIntrinsicMatrix());
    cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat rvec, tvec, inliers;
    bool ok = cv::solvePnPRansac(obj, img, K_cv, dist, rvec, tvec, false, 200,
                                 8.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
    if (!ok || inliers.rows < config_.min_pnp_inliers) {
        LOG(INFO) << "PnP RANSAC failed, inliers="
                  << (ok ? inliers.rows : 0);
        return false;
    }

    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    *T_cw_query = SE3(SO3(toMat33(R_cv)), toVec3(tvec));
    LOG(INFO) << "PnP inliers " << inliers.rows << "/" << obj.size();
    return true;
}

}  // namespace stereo_vslam
