#include "likelihood_field.h"
#include <glog/logging.h>

namespace zqx {

void LikelihoodField::BuildModel() {
  const int range = 20;
  for (int x = -range; x <= range; ++x) {
    for (int y = -range; y <= range; ++y) {
      model_.emplace_back(x, y, std::sqrt((x * x) + (y * y)));
    }
  }
}

void LikelihoodField::SetSourceScan(Scan2d::Ptr scan) { source_ = scan; }

void LikelihoodField::SetTargetScan(Scan2d::Ptr scan) {
  target_ = scan;

  field_ = cv::Mat(1000, 1000, CV_32F, 30.0);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
      continue;
    }

    double real_angle = scan->angle_min + i * scan->angle_increment;
    double x = scan->ranges[i] * std::cos(real_angle) * resolution_ + 500;
    double y = scan->ranges[i] * std::sin(real_angle) * resolution_ + 500;

    for (auto& model_pt : model_) {
      int xx = int(x + model_pt.dx_);
      int yy = int(y + model_pt.dy_);
      if (xx >= 0 && x < field_.cols && yy >= 0 && y < field_.rows &&
          field_.at<float>(yy, xx) > model_pt.residual_) {
        field_.at<float>(yy, xx) = model_pt.residual_;
      }
    }
  }
}

bool LikelihoodField::AlignGaussNewton(SE2& init_pose) {
  int iterations = 10;
  double cost = 0, lastcost = 0;
  SE2 current_pose = init_pose;
  const int min_effect_pts = 20;
  const int image_boarder = 20;

  has_outside_pts_ = false;
  for (int iter = 0; iter < iterations; ++iter) {
    Mat3d H = Mat3d::Zero();
    Vec3d b = Vec3d::Zero();
    cost = 0;
    int effective_num = 0;

    if (source_ == nullptr) {
      LOG(INFO) << "source scan is empty";
      return false;
    }

    for (size_t i = 0; i < source_->ranges.size(); ++i) {
      float r = source_->ranges[i];
      if (r < source_->range_min || r > source_->range_max) {
        continue;
      }
      float angle = source_->angle_min + i * source_->angle_increment;
      if (angle < source_->angle_min + 30 * M_PI / 180.0 ||
          angle > source_->angle_max - 30 * M_PI / 180.0) {
        continue;
      }

      float theta = current_pose.so2().log();
      Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));

      Vec2i pf = (pw * resolution_ + Vec2d(500, 500)).cast<int>();
      if (pf[0] >= image_boarder && pf[0] < field_.cols - image_boarder && pf[1] >= image_boarder &&
          pf[1] < field_.rows - image_boarder) {
        effective_num++;

        float dx = 0.5 * (field_.at<float>(pf[1], pf[0] + 1) - field_.at<float>(pf[1], pf[0] - 1));
        float dy = 0.5 * (field_.at<float>(pf[1] + 1, pf[0]) - field_.at<float>(pf[1] - 1, pf[0]));

        Vec3d J;
        J << resolution_ * dx, resolution_ * dy,
            -resolution_ * dx * r * std::sin(angle + theta) +
                resolution_ * dy * r * std::cos(angle + theta);
        H += J * J.transpose();
        float e = field_.at<float>(pf[1], pf[0]);
        b += -J * e;
        cost += e * e;
      } else {
        has_outside_pts_ = true;
      }
    }

    if (effective_num < min_effect_pts) {
      return false;
    }

    Vec3d dx = H.ldlt().solve(b);
    if (isnan(dx[0])) {
      break;
    }

    cost /= effective_num;
    if (iter > 0 && cost >= lastcost) {
      break;
    }

    // LOG(INFO) << "iter " << iter << " cost = " << cost << ", effect num: " << effective_num;

    current_pose.translation() += dx.head<2>();
    current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
    lastcost = cost;
  }

  init_pose = current_pose;
  return true;
}

cv::Mat LikelihoodField::GetFieldImage() {
  cv::Mat image(field_.rows, field_.cols, CV_8UC3);
  for (int x = 0; x < field_.cols; ++x) {
    for (int y = 0; y < field_.rows; ++y) {
      float r = field_.at<float>(y, x) * 255.0 / 30.0;
      image.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(r), uchar(r), uchar(r));
    }
  }
  return image;
}

void LikelihoodField::SetFieldImageFromOccuMap(const cv::Mat& occu_map) {
  const int boarder = 25;
  field_ = cv::Mat(1000, 1000, CV_32F, 30.0);
  for (int x = boarder; x < occu_map.cols - boarder; ++x) {
    for (int y = boarder; y < occu_map.rows - boarder; ++y) {
      if (occu_map.at<uchar>(y, x) < 127) {
        for (auto& model_pt : model_) {
          int xx = int(x + model_pt.dx_);
          int yy = int(y + model_pt.dy_);
          if (xx >= 0 && x < field_.cols && yy >= 0 && yy < field_.rows &&
              field_.at<float>(yy, xx) > model_pt.residual_) {
            field_.at<float>(yy, xx) = model_pt.residual_;
          }
        }
      }
    }
  }
}

}  // namespace zqx