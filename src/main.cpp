#include <gflags/gflags.h>
#include "visual_slam/visual_slam.h"

DEFINE_string(config_file, "./config/config.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    visual_slam::VisualSLAM::Ptr vslam(
        new visual_slam::VisualSLAM(FLAGS_config_file));
    vslam->run();

    return 0;
}
