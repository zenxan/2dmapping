#pragma once

#include <opencv2/core/core.hpp>
#include "common/eigen_types.h"
#include "common/lidar_utils.h"

namespace zqx {

void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color,
                     int image_size = 800, float resolution = 20.0, const SE2& pose_submap = SE2());

}