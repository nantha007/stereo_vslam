#include "stereo_vslam/core/landmark.h"
#include "stereo_vslam/core/feature.h"

namespace stereo_vslam {

Landmark::Ptr Landmark::create() {
    static Id factory_id = 0;
    Landmark::Ptr new_landmark(new Landmark);
    new_landmark->id = factory_id++;
    return new_landmark;
}

void Landmark::removeObservation(std::shared_ptr<Feature> feature) {
    bool removed = false;
    for (auto iter = observations_.begin(); iter != observations_.end();
         iter++) {
        if (iter->lock() == feature) {
            observations_.erase(iter);
            num_observations_--;
            removed = true;
            break;
        }
    }
    if (removed && feature) feature->resetLandmark();
}

}  // namespace stereo_vslam
