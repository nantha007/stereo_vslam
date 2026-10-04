#include "stereo_vslam/backend/optimizer.h"

#include <cmath>
#include <map>
#include <utility>

#include <glog/logging.h>

#include "stereo_vslam/backend/g2o_types.h"
#include "stereo_vslam/core/conversion.h"
#include "stereo_vslam/core/landmark.h"

namespace stereo_vslam {
namespace optimizer {

namespace {

constexpr double chi2_threshold = 5.991;
const double huber_delta = std::sqrt(chi2_threshold);

}  // namespace

int optimizePose(Frame &frame, const Camera &camera_left, const MapState &state,
                SE3 *pose) {
    using BlockSolverType = g2o::BlockSolver_6_3;
    using LinearSolverType =
        g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(
            std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer solver_graph;
    solver_graph.setAlgorithm(solver);

    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(frame.getPose());
    solver_graph.addVertex(vertex_pose);

    Mat33 K = camera_left.getIntrinsicMatrix();

    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < frame.features_left.size(); ++i) {
        auto landmark = frame.features_left[i]->getLandmark();
        if (landmark) {
            features.push_back(frame.features_left[i]);
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(landmarkPosition(state, *landmark), K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(toVec2(frame.features_left[i]->keypoint.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            auto kernel = new g2o::RobustKernelHuber;
            kernel->setDelta(huber_delta);
            edge->setRobustKernel(kernel);
            edges.push_back(edge);
            solver_graph.addEdge(edge);
            index++;
        }
    }

    int num_outliers = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(frame.getPose());
        solver_graph.initializeOptimization();
        solver_graph.optimize(10);
        num_outliers = 0;

        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->isOutlier()) {
                e->computeError();
            }
            if (e->chi2() > chi2_threshold) {
                features[i]->setOutlier(true);
                e->setLevel(1);
                num_outliers++;
            } else {
                features[i]->setOutlier(false);
                e->setLevel(0);
            }

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outliers/inliers in pose estimation: " << num_outliers << "/"
              << features.size() - num_outliers;

    const SE3 estimated = vertex_pose->estimate();
    LOG(INFO) << "Current Pose = \n" << estimated.matrix();
    if (pose) *pose = estimated;
    return static_cast<int>(features.size()) - num_outliers;
}

struct LocalBAGraph::Impl {
    g2o::SparseOptimizer solver_graph;
    std::map<Id, VertexPose *> pose_vertices;
    std::map<Id, VertexXYZ *> landmark_vertices;
    std::vector<std::pair<EdgeProjection *, Feature::Ptr>> edges_and_features;
};

LocalBAGraph::LocalBAGraph(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

LocalBAGraph::LocalBAGraph(LocalBAGraph &&) noexcept = default;
LocalBAGraph &LocalBAGraph::operator=(LocalBAGraph &&) noexcept = default;
LocalBAGraph::~LocalBAGraph() = default;

std::unique_ptr<LocalBAGraph> buildLocalBAGraph(
    const MapState &state, const Camera &camera_left,
    const Camera &camera_right) {
    const Map::KeyframeMap &keyframes = state.active_keyframes;
    const Map::LandmarkMap &landmarks = state.active_landmarks;
    if (keyframes.size() < 2) return nullptr;

    using BlockSolverType = g2o::BlockSolver_6_3;
    using LinearSolverType =
        g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(
            std::make_unique<LinearSolverType>()));
    auto impl = std::unique_ptr<LocalBAGraph::Impl>(new LocalBAGraph::Impl);
    g2o::SparseOptimizer &solver_graph = impl->solver_graph;
    solver_graph.setAlgorithm(solver);

    std::map<Id, VertexPose *> &pose_vertices = impl->pose_vertices;
    Id newest_keyframe_id = 0;
    Id second_newest_keyframe_id = 0;
    bool has_newest = false;
    bool has_second = false;
    for (auto &keyframe : keyframes) {
        auto frame = keyframe.second;
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(frame->keyframe_id);
        auto pose = state.keyframe_poses.find(frame->keyframe_id);
        vertex_pose->setEstimate(pose != state.keyframe_poses.end()
                                     ? pose->second
                                     : frame->getPose());
        solver_graph.addVertex(vertex_pose);
        const Id id = frame->keyframe_id;
        if (!has_newest || id > newest_keyframe_id) {
            if (has_newest) {
                second_newest_keyframe_id = newest_keyframe_id;
                has_second = true;
            }
            newest_keyframe_id = id;
            has_newest = true;
        } else if (!has_second || id > second_newest_keyframe_id) {
            second_newest_keyframe_id = id;
            has_second = true;
        }
        pose_vertices.insert({id, vertex_pose});
    }
    // Free the two newest poses. With exactly two keyframes, fix the older
    // one so the gauge stays constrained.
    const bool fix_all_but_newest = keyframes.size() == 2;
    for (auto &v : pose_vertices) {
        if (v.first == newest_keyframe_id) continue;
        if (!fix_all_but_newest && has_second &&
            v.first == second_newest_keyframe_id)
            continue;
        v.second->setFixed(true);
    }

    std::map<Id, VertexXYZ *> &landmark_vertices = impl->landmark_vertices;

    Mat33 K = camera_left.getIntrinsicMatrix();
    SE3 extrinsic_left = camera_left.getExtrinsic();
    SE3 extrinsic_right = camera_right.getExtrinsic();

    int index = 1;
    std::vector<std::pair<EdgeProjection *, Feature::Ptr>> &edges_and_features =
        impl->edges_and_features;

    for (auto &landmark : landmarks) {
        Id landmark_id = landmark.second->id;
        auto observations = landmark.second->getObservations();
        if (observations.size() <= 1) continue;
        for (auto &obs : observations) {
            if (obs.lock() == nullptr) continue;
            auto feature = obs.lock();
            if (feature->isOutlier() || feature->frame.lock() == nullptr)
                continue;

            auto frame = feature->frame.lock();
            EdgeProjection *edge = nullptr;
            if (feature->is_on_left_image) {
                edge = new EdgeProjection(K, extrinsic_left);
            } else {
                edge = new EdgeProjection(K, extrinsic_right);
            }

            if (landmark_vertices.find(landmark_id) ==
                    landmark_vertices.end()) {
                VertexXYZ *v = new VertexXYZ;
                auto position = state.landmark_positions.find(landmark_id);
                v->setEstimate(position != state.landmark_positions.end()
                                   ? position->second
                                   : landmark.second->getPosition());
                v->setId(landmark_id + newest_keyframe_id + 1);
                v->setMarginalized(true);
                landmark_vertices.insert({landmark_id, v});
                solver_graph.addVertex(v);
            }

            if (pose_vertices.find(frame->keyframe_id) != pose_vertices.end() &&
                landmark_vertices.find(landmark_id) !=
                    landmark_vertices.end()) {
                edge->setId(index);
                edge->setVertex(0, pose_vertices.at(frame->keyframe_id));
                edge->setVertex(1, landmark_vertices.at(landmark_id));
                edge->setMeasurement(toVec2(feature->keypoint.pt));
                edge->setInformation(Mat22::Identity());
                auto kernel = new g2o::RobustKernelHuber();
                kernel->setDelta(huber_delta);
                edge->setRobustKernel(kernel);
                edges_and_features.emplace_back(edge, feature);
                solver_graph.addEdge(edge);
                index++;
            } else {
                delete edge;
            }
        }
    }

    if (edges_and_features.empty()) return nullptr;
    return std::unique_ptr<LocalBAGraph>(new LocalBAGraph(std::move(impl)));
}

bool solveLocalBA(LocalBAGraph &graph, LocalBAResult *result) {
    if (!result || !graph.impl_) return false;

    g2o::SparseOptimizer &solver_graph = graph.impl_->solver_graph;
    auto &pose_vertices = graph.impl_->pose_vertices;
    auto &landmark_vertices = graph.impl_->landmark_vertices;
    auto &edges_and_features = graph.impl_->edges_and_features;
    double chi2_threshold_local = chi2_threshold;

    solver_graph.initializeOptimization();
    solver_graph.optimize(5);

    int iteration = 0;
    while (iteration < 2) {
        int num_outliers = 0;
        int num_inliers = 0;
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_threshold_local) {
                num_outliers++;
            } else {
                num_inliers++;
            }
        }
        double inlier_ratio =
            num_inliers / double(num_inliers + num_outliers);
        if (inlier_ratio > 0.5) {
            break;
        } else {
            chi2_threshold_local *= 2;
            iteration++;
        }
    }

    result->outlier_features.clear();
    result->inlier_features.clear();
    for (auto &ef : edges_and_features) {
        if (ef.first->chi2() > chi2_threshold_local) {
            result->outlier_features.push_back(ef.second);
        } else {
            result->inlier_features.push_back(ef.second);
        }
    }
    result->num_outliers = static_cast<int>(result->outlier_features.size());
    result->num_inliers = static_cast<int>(result->inlier_features.size());

    for (auto &v : pose_vertices) {
        result->keyframe_poses[v.first] = v.second->estimate();
    }
    for (auto &v : landmark_vertices) {
        result->landmark_positions[v.first] = v.second->estimate();
    }
    return true;
}

PoseGraphResult optimizePoseGraph(const PoseMap &poses,
                                  const Map::EdgeList &edges) {
    PoseGraphResult result;
    if (poses.size() < 2 || edges.empty()) return result;

    using BlockSolverType = g2o::BlockSolver_6_3;
    using LinearSolverType =
        g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(
            std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer solver_graph;
    solver_graph.setAlgorithm(solver);

    std::map<Id, VertexPose *> pose_vertices;
    for (auto &kv : poses) {
        VertexPose *v = new VertexPose();
        v->setId(static_cast<int>(kv.first));
        v->setEstimate(kv.second);
        if (kv.first == 0) v->setFixed(true);
        solver_graph.addVertex(v);
        pose_vertices[kv.first] = v;
        result.poses_before[kv.first] = kv.second;
        if (pose_vertices.size() == 1 || kv.first > result.newest_keyframe_id) {
            result.newest_keyframe_id = kv.first;
        }
    }
    if (pose_vertices.size() < 2) return result;

    int num_edges = 0;
    for (const auto &edge : edges) {
        auto it_from = pose_vertices.find(edge.from_keyframe_id);
        auto it_to = pose_vertices.find(edge.to_keyframe_id);
        if (it_from == pose_vertices.end() || it_to == pose_vertices.end()) continue;
        EdgeRelativePose *e = new EdgeRelativePose();
        e->setId(num_edges++);
        e->setVertex(0, it_from->second);
        e->setVertex(1, it_to->second);
        e->setMeasurement(edge.T_from_to);
        e->setInformation(edge.information);
        solver_graph.addEdge(e);
    }
    if (num_edges == 0) return result;

    solver_graph.initializeOptimization();
    solver_graph.optimize(20);

    for (auto &kv : pose_vertices) {
        result.poses_after[kv.first] = kv.second->estimate();
    }
    result.is_valid = true;
    return result;
}

}  // namespace optimizer
}  // namespace stereo_vslam
