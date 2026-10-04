#include "stereo_vslam/viz/viewer.h"
#include "stereo_vslam/core/feature.h"
#include "stereo_vslam/core/frame.h"
#include "stereo_vslam/core/landmark.h"

#include <algorithm>
#include <cassert>
#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

namespace stereo_vslam {

namespace {

void appendXYZ(std::vector<float> &dst, const Vec3 &p) {
    dst.push_back(static_cast<float>(p[0]));
    dst.push_back(static_cast<float>(p[1]));
    dst.push_back(static_cast<float>(p[2]));
}

void appendFrustum(std::vector<float> &dst, const SE3 &T_wc) {
    // Same camera model as Viewer::drawFrame.
    const float sz = 1.0f;
    const float fx = 400.f;
    const float fy = 400.f;
    const float cx = 512.f;
    const float cy = 384.f;
    const float width = 1080.f;
    const float height = 768.f;
    const Vec3 cam[5] = {
        Vec3::Zero(),
        Vec3(sz * (0.f - cx) / fx, sz * (0.f - cy) / fy, sz),
        Vec3(sz * (0.f - cx) / fx, sz * (height - 1.f - cy) / fy, sz),
        Vec3(sz * (width - 1.f - cx) / fx, sz * (height - 1.f - cy) / fy, sz),
        Vec3(sz * (width - 1.f - cx) / fx, sz * (0.f - cy) / fy, sz),
    };
    const int edges[8][2] = {{0, 1}, {0, 2}, {0, 3}, {0, 4},
                             {4, 3}, {3, 2}, {2, 1}, {1, 4}};
    for (const auto &edge : edges) {
        appendXYZ(dst, T_wc * cam[edge[0]]);
        appendXYZ(dst, T_wc * cam[edge[1]]);
    }
}

void drawFloats(GLenum mode, const std::vector<float> &xyz, float r, float g,
                float b, float size, bool points) {
    if (xyz.size() < 3) return;
    glColor3f(r, g, b);
    if (points) {
        glPointSize(size);
    } else {
        glLineWidth(size);
    }
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, xyz.data());
    glDrawArrays(mode, 0, static_cast<GLsizei>(xyz.size() / 3));
    glDisableClientState(GL_VERTEX_ARRAY);
}

}  // namespace

Viewer::Viewer(Map::Ptr map) : map_(std::move(map)) {}

void Viewer::start() {
    if (viewer_thread_.joinable()) return;
    viewer_thread_ = std::jthread([this](std::stop_token token) {
        runViewer(std::move(token));
    });
}

Viewer::~Viewer() { stop(); }

void Viewer::stop() {
    viewer_thread_.request_stop();
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }
}

void Viewer::onFrameTracked(Frame::Ptr frame) {
    addCurrentFrame(std::move(frame));
}

void Viewer::onMapUpdated() { updateMap(); }

void Viewer::onLoopClosed(const PoseMap &poses_before,
                          const PoseMap &poses_after) {
    correctTrajectory(poses_before, poses_after);
    updateMap();
}

void Viewer::addCurrentFrame(Frame::Ptr current_frame) {
    std::shared_ptr<FrameView> view;
    bool is_keyframe = false;
    Id keyframe_id = 0;
    if (current_frame) {
        view = std::make_shared<FrameView>();
        std::vector<cv::Point2f> tracked;
        cv::Mat gray;
        view->T_cw = current_frame->getPose();
        is_keyframe = current_frame->is_keyframe;
        keyframe_id = current_frame->keyframe_id;
        if (!current_frame->image_left.empty()) {
            gray = current_frame->image_left;
            tracked.reserve(current_frame->features_left.size());
            for (const auto &feature : current_frame->features_left) {
                if (feature && feature->getLandmark()) {
                    tracked.push_back(feature->keypoint.pt);
                }
            }
        }
        if (!gray.empty()) {
            view->bgr = plotFrameImage(gray, tracked);
        }
    }

    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    frame_view_ = std::move(view);
    if (!current_frame) return;
    if (is_keyframe) {
        last_keyframe_id_ = keyframe_id;
        has_last_keyframe_ = true;
    }
    if (!has_last_keyframe_) return;

    TrajectoryNode node;
    node.ref_keyframe_id = last_keyframe_id_;
    node.T_cw = frame_view_->T_cw;
    trajectory_.push_back(node);

    auto xyz = std::make_shared<std::vector<float>>();
    if (trajectory_xyz_) *xyz = *trajectory_xyz_;
    appendXYZ(*xyz, node.T_cw.inverse().translation());
    trajectory_xyz_ = std::move(xyz);
}

void Viewer::correctTrajectory(const PoseMap &poses_before,
                                const PoseMap &poses_after) {
    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    auto xyz = std::make_shared<std::vector<float>>();
    xyz->reserve(trajectory_.size() * 3);
    for (auto &node : trajectory_) {
        auto it_before = poses_before.find(node.ref_keyframe_id);
        auto it_after = poses_after.find(node.ref_keyframe_id);
        if (it_before != poses_before.end() && it_after != poses_after.end()) {
            node.T_cw =
                node.T_cw * it_before->second.inverse() * it_after->second;
        }
        appendXYZ(*xyz, node.T_cw.inverse().translation());
    }
    trajectory_xyz_ = std::move(xyz);
}

std::shared_ptr<std::vector<float>> Viewer::buildTrajectoryXYZ() const {
    auto xyz = std::make_shared<std::vector<float>>();
    xyz->reserve(trajectory_.size() * 3);
    for (const auto &node : trajectory_) {
        appendXYZ(*xyz, node.T_cw.inverse().translation());
    }
    return xyz;
}

std::shared_ptr<Viewer::DrawCache> Viewer::buildDrawCache(const MapState &state) {
    auto cache = std::make_shared<DrawCache>();
    const Map::KeyframeIdPairs loop_pairs = state.loopKeyframePairs();

    struct KeyframeDraw {
        Id id = 0;
        SE3 T_cw;
        Vec3 t_wc = Vec3::Zero();
    };
    std::vector<KeyframeDraw, Eigen::aligned_allocator<KeyframeDraw>> draws;
    draws.reserve(state.keyframes.size());
    for (const auto &keyframe : state.keyframes) {
        if (!keyframe.second) continue;
        auto pose = state.keyframe_poses.find(keyframe.first);
        if (pose == state.keyframe_poses.end()) continue;
        KeyframeDraw draw;
        draw.id = keyframe.first;
        draw.T_cw = pose->second;
        draw.t_wc = draw.T_cw.inverse().translation();
        draws.push_back(draw);
    }
    std::sort(draws.begin(), draws.end(),
              [](const KeyframeDraw &a, const KeyframeDraw &b) {
                  return a.id < b.id;
              });

    std::unordered_set<Id> active_keyframe_ids;
    active_keyframe_ids.reserve(state.active_keyframes.size());
    for (const auto &keyframe : state.active_keyframes) {
        active_keyframe_ids.insert(keyframe.first);
    }

    cache->inactive_keyframes.reserve(draws.size() * 48);
    cache->active_keyframes.reserve(active_keyframe_ids.size() * 48);
    cache->odometry.reserve(draws.size() * 6);
    for (size_t i = 0; i < draws.size(); ++i) {
        const SE3 T_wc = draws[i].T_cw.inverse();
        auto &frustum = active_keyframe_ids.count(draws[i].id)
                            ? cache->active_keyframes
                            : cache->inactive_keyframes;
        appendFrustum(frustum, T_wc);
        if (i + 1 < draws.size()) {
            appendXYZ(cache->odometry, draws[i].t_wc);
            appendXYZ(cache->odometry, draws[i + 1].t_wc);
        }
    }

    std::unordered_map<Id, size_t> draw_index;
    draw_index.reserve(draws.size());
    for (size_t i = 0; i < draws.size(); ++i) {
        draw_index.emplace(draws[i].id, i);
    }
    cache->loops.reserve(loop_pairs.size() * 6);
    for (const auto &pair : loop_pairs) {
        auto from = draw_index.find(pair.first);
        auto to = draw_index.find(pair.second);
        if (from == draw_index.end() || to == draw_index.end()) continue;
        appendXYZ(cache->loops, draws[from->second].t_wc);
        appendXYZ(cache->loops, draws[to->second].t_wc);
    }

    std::unordered_set<Id> active_landmark_ids;
    for (const auto &landmark : state.active_landmarks) {
        if (landmark.second) {
            active_landmark_ids.insert(landmark.first);
        }
    }
    cache->inactive_landmarks.reserve(state.landmarks.size() * 3);
    cache->active_landmarks.reserve(active_landmark_ids.size() * 3);
    for (const auto &landmark : state.landmarks) {
        if (!landmark.second) continue;
        auto position = state.landmark_positions.find(landmark.first);
        if (position == state.landmark_positions.end()) continue;
        auto &dst = active_landmark_ids.count(landmark.first)
                        ? cache->active_landmarks
                        : cache->inactive_landmarks;
        appendXYZ(dst, position->second);
    }
    return cache;
}

void Viewer::updateMap() {
    assert(map_ != nullptr);
    auto state = map_->load();
    std::shared_ptr<DrawCache> cache;
    if (state) cache = buildDrawCache(*state);

    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    draw_cache_ = std::move(cache);
    trajectory_xyz_ = buildTrajectoryXYZ();
}

void Viewer::runViewer(std::stop_token token) {
    pangolin::CreateWindowAndBind("stereo_vslam", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &vis_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));

    const float green[3] = {0, 1, 0};
    std::shared_ptr<const FrameView> shown_frame;

    while (!pangolin::ShouldQuit() && !token.stop_requested()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::shared_ptr<const FrameView> frame;
        std::shared_ptr<const DrawCache> cache;
        std::shared_ptr<const std::vector<float>> trajectory;
        {
            std::unique_lock<std::mutex> lock(viewer_data_mutex_);
            frame = frame_view_;
            cache = draw_cache_;
            trajectory = trajectory_xyz_;
        }

        if (frame) {
            drawFrame(frame->T_cw, green);
            followCurrentFrame(vis_camera, frame->T_cw);
            if (frame != shown_frame && !frame->bgr.empty()) {
                cv::imshow("image", frame->bgr);
                shown_frame = frame;
            }
        }
        cv::waitKey(1);

        if (cache) {
            if (trajectory && trajectory->size() >= 6) {
                drawFloats(GL_LINE_STRIP, *trajectory, 0.2f, 0.2f, 0.2f, 2.f,
                           false);
            }
            drawFloats(GL_LINES, cache->odometry, 0.15f, 0.55f, 0.25f, 1.5f,
                       false);
            drawFloats(GL_LINES, cache->loops, 0.85f, 0.1f, 0.75f, 3.f,
                       false);
            drawFloats(GL_LINES, cache->inactive_keyframes, 0.f, 0.f, 1.f, 2.f,
                       false);
            drawFloats(GL_LINES, cache->active_keyframes, 1.f, 0.f, 0.f, 2.f,
                       false);
            drawFloats(GL_POINTS, cache->inactive_landmarks, 0.25f, 0.25f,
                       0.25f, 2.f, true);
            drawFloats(GL_POINTS, cache->active_landmarks, 1.f, 0.2f, 0.1f,
                       3.f, true);
        }

        pangolin::FinishFrame();
        usleep(33000);
    }

    LOG(INFO) << "Stopped viewer";
}

cv::Mat Viewer::plotFrameImage(const cv::Mat &gray,
                               const std::vector<cv::Point2f> &tracked_points) {
    cv::Mat img_out;
    cv::cvtColor(gray, img_out, cv::COLOR_GRAY2BGR);
    for (const auto &pt : tracked_points) {
        cv::circle(img_out, pt, 2, cv::Scalar(0, 250, 0), 2);
    }
    return img_out;
}

void Viewer::followCurrentFrame(pangolin::OpenGlRenderState &vis_camera,
                                const SE3 &T_cw) {
    SE3 T_wc = T_cw.inverse();
    pangolin::OpenGlMatrix m(T_wc.matrix());
    vis_camera.Follow(m, true);
}

void Viewer::drawFrame(const SE3 &T_cw, const float *color) {
    SE3 T_wc = T_cw.inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = T_wc.matrix().template cast<float>();
    glMultMatrixf((GLfloat *)m.data());

    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

}  // namespace stereo_vslam
