#include <gflags/gflags.h>
#include "stereo_vslam/visual_slam.h"

DEFINE_string(config_file, "./config/config.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    stereo_vslam::VisualSLAM::Ptr vslam(
        new stereo_vslam::VisualSLAM(FLAGS_config_file));
    vslam->run();

    return 0;
}
