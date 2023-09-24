#include "2dmapping/occupancy_map.h"
#include <execution>
#include "common/math_utils.h"
#include "common/sys_utils.h"

namespace zqx {

OccupancyMap::OccupancyMap() {
  BuildModel();
  occupancy_grid_ = cv::Mat(image_size_, image_size_, CV_8U, 127);
}

void OccupancyMap::BuildModel() {
  for (int x = -model_size_; x <= model_size_; x++) {
    for (int y = -model_size_; y <= model_size_; y++) {
      Model2Dpoint pt;
      pt.dx_ = x;
      pt.dy_ = y;
      pt.range_ = sqrt(x * x + y * y) * inv_resolution_;
      pt.angle_ = std::atan2(y, x);
      pt.angle_ = pt.angle_ > M_PI ? pt.angle_ - 2 * M_PI : pt.angle_;
      model_.emplace_back(pt);
    }
  }
}

void OccupancyMap::AddLidarFrame(std::shared_ptr<Frame> frame, GridMethod method) {
  auto& scan = frame->scan_;

  SE2 pose_in_submap = pose_.inverse() * frame->pose_;
  float theta = pose_in_submap.so2().log();
  has_outside_pts_ = false;

  std::set<Vec2i, sad::less_vec<2>> endpoints;

  for (size_t i = 0; i < scan->ranges.size(); i++) {
    if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
      continue;
    }

    double real_angle = scan->angle_min + i * scan->angle_increment;
    double x = scan->ranges[i] * std::cos(real_angle);
    double y = scan->ranges[i] * std::sin(real_angle);

    endpoints.emplace(World2Image(frame->pose_ * Vec2d(x, y)));
  }

  if (method == GridMethod::MODEL_POINTS) {
    std::for_each(std::execution::par_unseq, model_.begin(), model_.end(),
                  [&](const Model2Dpoint& pt) {
                    Vec2i pos_in_image = World2Image(frame->pose_.translation());
                    Vec2i pw = pos_in_image + Vec2i(pt.dx_, pt.dy_);

                    if (pt.range_ < closest_th_) {
                      SetPoint(pw, false);
                      return;
                    }

                    double angle = pt.angle_ - theta;
                    double range = FindRangeInAngle(angle, scan);

                    if (range < scan->range_min || range > scan->range_max) {
                      if (pt.range_ < endpoint_close_th_) {
                        SetPoint(pw, false);
                      }
                      return;
                    }

                    if (range > pt.range_ && endpoints.find(pw) == endpoints.end()) {
                      SetPoint(pw, false);
                    }
                  });
  }
  std::for_each(endpoints.begin(), endpoints.end(), [this](const auto& pt) { SetPoint(pt, true); });
}

double OccupancyMap::FindRangeInAngle(double angle, Scan2d::Ptr scan) {
  sad::math::KeepAngleInPI(angle);
  if (angle < scan->angle_min || angle > scan->angle_max) {
    return 0.0;
  }

  int angle_index = int((angle - scan->angle_min) / scan->angle_increment);
  if (angle_index < 0 || angle_index >= scan->ranges.size()) {
    return 0.0;
  }

  int angle_index_p = angle_index + 1;

  double real_angle = angle;
  double range = 0;
  if (angle_index_p >= scan->ranges.size()) {
    range = scan->ranges[angle_index];
  } else {
    double s = ((angle - scan->angle_min) / scan->angle_increment) - angle_index;
    double range1 = scan->ranges[angle_index];
    double range2 = scan->ranges[angle_index_p];

    double real_angle1 = scan->angle_min + scan->angle_increment * angle_index;
    double real_angle2 = scan->angle_min + scan->angle_increment * angle_index_p;

    if (range2 < scan->range_min || range2 > scan->range_max) {
      range = range1;
      real_angle = real_angle1;
    } else if (range1 < scan->range_min || range1 > scan->range_max) {
      range = range2;
      real_angle = real_angle2;
    } else if (std::fabs(range1 - range2) > 0.3) {
      range = s > 0.5 ? range2 : range1;
      real_angle = s > 0.5 ? real_angle2 : real_angle1;
    } else {
      range = range1 * (1 - s) + range2 * s;
    }
  }
  return range;
}

void OccupancyMap::SetPoint(const Vec2i& pt, bool occupy) {
  int x = pt[0], y = pt[1];
  if (x < 0 || y < 0 || x >= occupancy_grid_.cols || y >= occupancy_grid_.rows) {
    if (occupy) {
      has_outside_pts_ = true;
    }
    return;
  }

  uchar value = occupancy_grid_.at<uchar>(y, x);
  if (occupy) {
    if (value > 117) {
      occupancy_grid_.ptr<uchar>(y)[x] -= 1;
    }
  } else {
    if (value < 137) {
      occupancy_grid_.ptr<uchar>(y)[x] += 1;
    }
  }
}

cv::Mat OccupancyMap::GetOccupancyGridBlackWhite() const {
  cv::Mat image(image_size_, image_size_, CV_8UC3);
  for (int x = 0; x < occupancy_grid_.cols; ++x) {
    for (int y = 0; y < occupancy_grid_.rows; ++y) {
      if (occupancy_grid_.at<uchar>(y, x) == 127) {
        image.at<cv::Vec3b>(y, x) = cv::Vec3b(127, 127, 127);
      } else if (occupancy_grid_.at<uchar>(y, x) < 127) {
        image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
      } else if (occupancy_grid_.at<uchar>(y, x) > 127) {
        image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
      }
    }
  }
  return image;
}

}  // namespace zqx