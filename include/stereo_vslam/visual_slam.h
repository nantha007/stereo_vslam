#pragma once

#include <memory>
#include <string>

#include "stereo_vslam/backend/local_mapper.h"
#include "stereo_vslam/core/map.h"
#include "stereo_vslam/frontend/tracker.h"
#include "stereo_vslam/io/dataset.h"
#include "stereo_vslam/loop/loop_closure.h"
#include "stereo_vslam/loop/place_recognizer.h"
#include "stereo_vslam/viz/viewer.h"

namespace stereo_vslam {

/**
 * Public interface for visual SLAM.
 * Builds the modules and passes each frame through them: tracking, keyframe
 * insertion, local BA, loop detection, and visualization.
 */
class VisualSLAM {
   public:
    using Ptr = std::shared_ptr<VisualSLAM>;

    /** @brief Build the system from a YAML config file. */
    explicit VisualSLAM(const std::string &config_path);

    /** @brief Stop loop closure and the viewer. */
    ~VisualSLAM();

    /**
     * @brief Run SLAM on the dataset until it is exhausted or tracking fails.
     */
    void run();

    /**
     * @brief Track the next dataset frame and insert it when it is a keyframe.
     * @return False when the dataset is missing or exhausted.
     */
    bool processFrame();

   private:
    Map::Ptr map_ = nullptr;
    Tracker::Ptr tracker_ = nullptr;
    LocalMapper::Ptr local_mapper_ = nullptr;
    PlaceRecognizer::Ptr place_recognizer_ = nullptr;
    LoopClosure::Ptr loop_closure_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    // dataset
    Dataset::Ptr dataset_ = nullptr;
};
}  // namespace stereo_vslam
