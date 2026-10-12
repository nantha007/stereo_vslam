#pragma once

#include <memory>
#include <string>
#include <vector>

#include "visual_slam/core/camera.h"
#include "visual_slam/core/frame.h"

namespace visual_slam {

/**
 * Dataset loader
 * Pass the dataset path in the constructor (config key dataset.dataset_path).
 */
class Dataset {
   public:
    using Ptr = std::shared_ptr<Dataset>;

    /** @brief Load cameras from @p dataset_path. */
    Dataset(const std::string &dataset_path);

    /**
     * @brief Create the next frame containing the stereo images.
     * @return The frame, or null when the sequence is exhausted.
     */
    Frame::Ptr nextFrame();

    /** @brief Camera at @p camera_id. */
    Camera::Ptr getCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

   private:
    std::string dataset_path_;
    int current_image_index_ = 0;

    std::vector<Camera::Ptr> cameras_;
};
}  // namespace visual_slam
