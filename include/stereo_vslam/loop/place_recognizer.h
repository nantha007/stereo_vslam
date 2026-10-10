#pragma once

#include <memory>
#include <string>

#include <fbow/fbow.h>

#include "stereo_vslam/core/config.h"
#include "stereo_vslam/core/frame.h"

namespace stereo_vslam {

/**
 * PlaceRecognizer
 * Owns the FBoW vocabulary and fills the bag-of-words data of keyframes.
 */
class PlaceRecognizer {
   public:
    using Ptr = std::shared_ptr<PlaceRecognizer>;

    /**
     * @brief Load the FBoW vocabulary at @p vocabulary_path.
     * @param type Descriptor pipeline. The vocabulary byte size must match it.
     */
    PlaceRecognizer(const std::string &vocabulary_path, FeatureType type);

    /**
     * @brief Fill the bag-of-words fields of @p frame from its left-image
     * descriptors.
     *
     * Does nothing when the bag of words is already filled.
     */
    void computeBoW(Frame &frame) const;

   private:
    std::unique_ptr<fbow::Vocabulary> vocabulary_;
};

}  // namespace stereo_vslam
