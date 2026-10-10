#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

#include "stereo_vslam/core/camera.h"
#include "stereo_vslam/core/types.h"
#include "stereo_vslam/core/frame.h"
#include "stereo_vslam/core/map.h"
#include "stereo_vslam/core/config.h"
#include "stereo_vslam/core/feature.h"
#include "stereo_vslam/core/landmark.h"

namespace stereo_vslam {

class LightGlue;

/**
 * LoopClosure
 * Detects loops on a dedicated thread after a new keyframe is inserted
 * and adds a loop edge to the map when one is accepted.
 */
class LoopClosure {
   public:
    using Ptr = std::shared_ptr<LoopClosure>;
    /// Called after the correction is published. Empty means no listener.
    using LoopClosedCallback = std::function<void(const PoseMap &poses_before,
                                                  const PoseMap &poses_after)>;

    /**
     * @brief Build loop closure.
     * @param on_loop_closed Called after a correction is published. Empty means
     * no listener.
     * @param light_glue SuperPoint matcher. Empty keeps Hamming kNN.
     */
    LoopClosure(const LoopClosureConfig &config, Map::Ptr map,
                Camera::Ptr camera_left,
                LoopClosedCallback on_loop_closed = {},
                std::shared_ptr<LightGlue> light_glue = nullptr);

    /** @brief Stop the loop-closure thread. */
    ~LoopClosure();

    /** @brief Start the loop-closure thread. */
    void start();

    /**
     * @brief Hand the loop thread the latest keyframe.
     *
     * A keyframe already waiting is replaced; only the newest one is detected.
     */
    void submitKeyframe(Frame::Ptr keyframe);

    /** @brief Stop the loop-closure thread. */
    void stop();

   private:
    /** @brief Wait for keyframes and run detection until @p token is stopped. */
    void runLoop(std::stop_token token);

    /** @brief Search for a loop against @p query and publish one if it is accepted. */
    void detect(Frame::Ptr query);

    /**
     * @brief Solve the pose graph and publish the correction.
     *
     * Retries once when the publish loses the revision race. The loop
     * measurement stays the PnP relative that was accepted.
     * @return False when the correction is not published.
     */
    bool commitLoop(const PoseGraphEdge &edge);

    /**
     * @brief Publish @p edge against the current snapshot.
     * @param stale Set when the publish loses the revision race.
     * @return True when the correction is published.
     */
    bool closeLoop(const PoseGraphEdge &edge, bool *stale);

    /**
     * @brief Keyframes near @p query, excluding the most recent ones.
     * @param poses Keyframe poses in T_cw.
     */
    std::vector<Frame::Ptr> spatialCandidates(const Frame::Ptr &query,
                                              const Map::KeyframeMap &keyframes,
                                              const PoseMap &poses);

    /**
     * @brief Estimate the query pose from descriptor matches against @p match.
     * @param T_cw_query Receives the verified pose in T_cw.
     * @return False when PnP has fewer inliers than the configured minimum.
     */
    bool estimateLoopPose(const Frame::Ptr &query, const Frame::Ptr &match,
                          const MapState &state, SE3 *T_cw_query);

    LoopClosureConfig config_;
    Map::Ptr map_;
    Camera::Ptr camera_left_ = nullptr;
    LoopClosedCallback on_loop_closed_;
    std::shared_ptr<LightGlue> light_glue_;

    std::jthread loop_thread_;
    std::mutex pending_mutex_;
    std::condition_variable_any keyframe_cv_;
    bool has_new_keyframe_ = false;
    Frame::Ptr pending_keyframe_ = nullptr;

    Id last_loop_keyframe_id_ = 0;
    std::set<std::pair<Id, Id>> closed_pairs_;
};

}  // namespace stereo_vslam
