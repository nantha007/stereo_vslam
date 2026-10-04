#pragma once

#include <memory>
#include <optional>

#include "stereo_vslam/core/camera.h"
#include "stereo_vslam/core/config.h"
#include "stereo_vslam/core/frame.h"
#include "stereo_vslam/core/map.h"
#include "stereo_vslam/core/landmark.h"
#include "stereo_vslam/frontend/tracker.h"

namespace stereo_vslam {

/**
 * LocalMapper
 * Adds keyframes prepared by the tracker to the map, keeps the active
 * window bounded, and refines it with local bundle adjustment.
 */
class LocalMapper {
   public:
    using Ptr = std::shared_ptr<LocalMapper>;

    /** @brief Build a mapper that publishes keyframes into @p map. */
    LocalMapper(const LocalMapperConfig &config, Map::Ptr map,
                Camera::Ptr camera_left, Camera::Ptr camera_right);

    /**
     * @brief Publish a keyframe, its new landmarks, and the odometry edge.
     *
     * Trims the active window in the snapshot. Observations and image release
     * run only after publish succeeds.
     * @param base Snapshot this insert is based on.
     * @param keyframe Keyframe to insert. Its id is already assigned.
     * @param result New landmarks and the previous keyframe.
     * @return False when @p base is no longer the current revision.
     */
    bool insertKeyframe(const std::shared_ptr<const MapState> &base,
                        const Frame::Ptr &keyframe, const TrackResult &result);

    /**
     * @brief Run reprojection bundle adjustment on the active window.
     *
     * Skipped until a second keyframe exists. The result is dropped when
     * another publish wins the revision race.
     */
    void runLocalBA();

   private:
    /**
     * @brief Register each feature of @p keyframe that has a landmark as an
     * observation of that landmark.
     */
    void registerObservations(const Frame::Ptr &keyframe);

    /**
     * @brief Remove one active keyframe from @p draft when the window is over
     * its limit.
     * @param keyframe Keyframe just inserted. It is not the one removed.
     * @return The removed keyframe, or empty when the window is within its
     * limit.
     */
    std::optional<Frame::Ptr> trimActiveWindow(MapState &draft,
                                               const Frame::Ptr &keyframe);

    /**
     * @brief Drop the observations and images of a keyframe already removed
     * from the active window.
     */
    void releaseDeactivatedKeyframe(const Frame::Ptr &keyframe);

    LocalMapperConfig config_;
    Map::Ptr map_;
    Camera::Ptr camera_left_;
    Camera::Ptr camera_right_;
};

}  // namespace stereo_vslam
