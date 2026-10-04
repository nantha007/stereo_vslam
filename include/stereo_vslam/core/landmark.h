#pragma once

#include <list>
#include <memory>

#include "stereo_vslam/core/types.h"

namespace stereo_vslam {

struct Frame;

struct Feature;

/**
 * Landmark class
 * Features become landmarks after triangulation.
 * position_ is private until the landmark is published. After that, the map
 * snapshot holds the position. observations_ are updated on the main thread.
 */
struct Landmark {
   public:
    using Ptr = std::shared_ptr<Landmark>;
    Id id = 0;  // ID

    /** @brief Create an empty landmark. The id is assigned by create(). */
    Landmark() {}

    /** @brief Private position in the world frame. */
    Vec3 getPosition() const { return position_; }

    /** @brief Set the private position. @p new_position is in the world frame. */
    void setPosition(const Vec3 &new_position) { position_ = new_position; }

    /** @brief Record @p feature as an observation of this landmark. */
    void addObservation(std::shared_ptr<Feature> feature) {
        observations_.push_back(feature);
        num_observations_++;
    }

    /** @brief Drop @p feature and clear its landmark link. */
    void removeObservation(std::shared_ptr<Feature> feature);

    /**
     * @brief Remember the keyframe that created this landmark.
     *
     * Pose-graph correction uses it after the feature link is cleared when the
     * keyframe leaves the local window. Later calls do nothing.
     */
    void setRefKeyframeId(Id keyframe_id) {
        if (has_ref_keyframe_) return;
        ref_keyframe_id_ = keyframe_id;
        has_ref_keyframe_ = true;
    }

    /** @brief True after setRefKeyframeId() has been called. */
    bool hasRefKeyframe() const { return has_ref_keyframe_; }

    /** @brief Keyframe id stored by setRefKeyframeId(). */
    Id getRefKeyframeId() const { return ref_keyframe_id_; }

    /** @brief Observations recorded on the main thread. */
    std::list<std::weak_ptr<Feature>> getObservations() const {
        return observations_;
    }

    /** @brief Number of observations currently stored. */
    int getNumObservations() const { return num_observations_; }

    /** @brief Create a landmark and assign its id. */
    static Landmark::Ptr create();

   private:
    Vec3 position_ = Vec3::Zero();  // Position in world
    int num_observations_ = 0;  // current observations, not a lifetime count
    std::list<std::weak_ptr<Feature>> observations_;
    Id ref_keyframe_id_ = 0;
    bool has_ref_keyframe_ = false;
};
}  // namespace stereo_vslam
