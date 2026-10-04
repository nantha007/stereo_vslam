#pragma once

#include <chrono>
#include <iomanip>

#include <glog/logging.h>

namespace stereo_vslam {

/**
 * @brief Logs wall time of one step when the scope ends, including early
 * returns.
 *
 * When is_keyframe is set, the line also reports that flag as it stands at
 * scope exit.
 */
class ScopedLogTimer {
   public:
    /**
     * @brief Start timing @p name.
     * @param is_keyframe Optional flag read at destruction. Null omits it.
     */
    explicit ScopedLogTimer(const char *name, const bool *is_keyframe = nullptr)
        : name_(name),
          is_keyframe_(is_keyframe),
          start_(std::chrono::steady_clock::now()) {}

    /** @brief Log the elapsed time. */
    ~ScopedLogTimer() {
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - start_)
                              .count();
        if (is_keyframe_ != nullptr) {
            LOG(INFO) << name_ << " took " << std::fixed
                      << std::setprecision(3) << ms << " ms, "
                      << (*is_keyframe_ ? "keyframe" : "not keyframe");
        } else {
            LOG(INFO) << name_ << " took " << std::fixed
                      << std::setprecision(3) << ms << " ms";
        }
    }

    /** @brief Copying a timer is not supported. */
    ScopedLogTimer(const ScopedLogTimer &) = delete;

    /** @brief Copying a timer is not supported. */
    ScopedLogTimer &operator=(const ScopedLogTimer &) = delete;

   private:
    const char *name_;
    const bool *is_keyframe_;
    std::chrono::steady_clock::time_point start_;
};

}  // namespace stereo_vslam
