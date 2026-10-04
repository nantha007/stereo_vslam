#pragma once

#include <memory>
#include <mutex>
#include <stop_token>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "stereo_vslam/core/frame.h"
#include "stereo_vslam/core/types.h"
#include "stereo_vslam/core/map.h"

namespace pangolin {
class OpenGlRenderState;
}

namespace stereo_vslam {

/**
 * Visualization
 */
class Viewer {
   public:
    using Ptr = std::shared_ptr<Viewer>;

    /** @brief Build a viewer for @p map. */
    explicit Viewer(Map::Ptr map);

    /** @brief Stop the viewer thread. */
    ~Viewer();

    /** @brief Start the viewer thread. The map is already set. */
    void start();

    /** @brief Stop the viewer thread and wait for it to finish. */
    void stop();

    /** @brief Store the pose and annotated image of a tracked frame. */
    void onFrameTracked(Frame::Ptr frame);

    /** @brief Refresh the drawn map from the latest snapshot. */
    void onMapUpdated();

    /**
     * @brief Apply a loop correction to the trajectory and refresh the map.
     * @param poses_before Keyframe poses in T_cw before the correction.
     * @param poses_after Keyframe poses in T_cw after the correction.
     */
    void onLoopClosed(const PoseMap &poses_before, const PoseMap &poses_after);

   private:
    /** @brief Record @p current_frame for drawing and append it to the trajectory. */
    void addCurrentFrame(Frame::Ptr current_frame);

    /** @brief Rebuild the draw cache from the current map snapshot. */
    void updateMap();

    /**
     * @brief Move stored trajectory poses with each keyframe's loop correction.
     * @param poses_before Keyframe poses in T_cw before the correction.
     * @param poses_after Keyframe poses in T_cw after the correction.
     */
    void correctTrajectory(const PoseMap &poses_before,
                           const PoseMap &poses_after);

    struct TrajectoryNode {
        Id ref_keyframe_id = 0;
        SE3 T_cw;
    };

    /// Pose and annotated image of the latest tracked frame.
    struct FrameView {
        SE3 T_cw;
        cv::Mat bgr;
    };

    /// Map geometry in world-frame float xyz. Replaced when the map is published.
    struct DrawCache {
        std::vector<float> inactive_landmarks;
        std::vector<float> active_landmarks;
        std::vector<float> inactive_keyframes;
        std::vector<float> active_keyframes;
        std::vector<float> odometry;
        std::vector<float> loops;
    };

    /** @brief Run the Pangolin window until @p token is stopped. */
    void runViewer(std::stop_token token);

    /**
     * @brief Draw a camera frustum for @p T_cw.
     * @param color RGB color as three floats. Red is used when null.
     */
    void drawFrame(const SE3 &T_cw, const float *color);

    /** @brief Point the view camera along the current frame. */
    void followCurrentFrame(pangolin::OpenGlRenderState &vis_camera,
                            const SE3 &T_cw);

    /**
     * @brief Draw tracked points on a copy of @p gray.
     * @return BGR image for the frame panel.
     */
    cv::Mat plotFrameImage(const cv::Mat &gray,
                           const std::vector<cv::Point2f> &tracked_points);

    /** @brief World-frame geometry for landmarks, keyframes, and pose-graph edges. */
    static std::shared_ptr<DrawCache> buildDrawCache(const MapState &state);

    /**
     * @brief Camera centers from the stored trajectory.
     *
     * The caller holds viewer_data_mutex_.
     */
    std::shared_ptr<std::vector<float>> buildTrajectoryXYZ() const;

    Map::Ptr map_ = nullptr;

    std::jthread viewer_thread_;

    std::shared_ptr<const FrameView> frame_view_;
    std::shared_ptr<const DrawCache> draw_cache_;
    /// Published separately so a new frame appends one point without copying
    /// the landmark buffers.
    std::shared_ptr<const std::vector<float>> trajectory_xyz_;

    std::vector<TrajectoryNode, Eigen::aligned_allocator<TrajectoryNode>>
        trajectory_;
    Id last_keyframe_id_ = 0;
    bool has_last_keyframe_ = false;

    std::mutex viewer_data_mutex_;
};
}  // namespace stereo_vslam
