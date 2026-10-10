#include "stereo_vslam/io/dataset.h"
#include "stereo_vslam/core/frame.h"

#include <fstream>

#include <boost/format.hpp>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

namespace stereo_vslam {

Dataset::Dataset(const std::string &dataset_path)
    : dataset_path_(dataset_path) {
    // read camera intrinsics and extrinsics
    std::ifstream fin(dataset_path_ + "/calib.txt");
    CHECK(fin) << "cannot find " << dataset_path_ << "/calib.txt!";

    for (int i = 0; i < 4; ++i) {
        fin >> std::ws;
        fin.ignore(3);
        double projection_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> projection_data[k];
        }
        Mat33 K;
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;
        // K = K * 0.5;
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                          t.norm(), SE3(SO3(), t)));
        cameras_.push_back(new_camera);
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }
    fin.close();
    current_image_index_ = 0;
}

Frame::Ptr Dataset::nextFrame() {
    boost::format fmt("%s/image_%d/%06d.png");
    cv::Mat image_left, image_right;
    // read images
    image_left =
        cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
    image_right =
        cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);

    if (image_left.data == nullptr || image_right.data == nullptr) {
        LOG(WARNING) << "Could not find images at index " << current_image_index_;
        return nullptr;
    }

    auto new_frame = Frame::create();
    new_frame->image_left = image_left;
    new_frame->image_right = image_right;
    new_frame->image_size = image_left.size();
    current_image_index_++;
    return new_frame;
}

}  // namespace stereo_vslam
