#pragma once

#include <opencv2/core.hpp>
#include "2dmapping/frame.h"

namespace zqx {

class OccupancyMap {
 public:
  struct Model2Dpoint {
    int dx_ = 0;
    int dy_ = 0;
    double angle_ = 0;
    float range_ = 0;
  };

  enum class GridMethod {
    MODEL_POINTS,
    BRESENHAM,
  };

  OccupancyMap();

  void AddLidarFrame(std::shared_ptr<Frame> frame, GridMethod method = GridMethod::MODEL_POINTS);

  cv::Mat GetOccupancyGridBlackWhite() const;

  void SetPoint(const Vec2i& pt, bool occupy);

  void SetPose(const SE2& pose) { pose_ = pose; }

  bool HasOutsidePoints() const { return has_outside_pts_; }

  double FindRangeInAngle(double angle, Scan2d::Ptr scan);

  cv::Mat GetOccupancyGrid() const { return occupancy_grid_; }

 private:
  void BuildModel();

  template <class T>
  inline Vec2i World2Image(const Eigen::Matrix<T, 2, 1>& pt) {
    Vec2d pt_map = (pose_.inverse() * pt) * resolution_ + center_image_;
    int x = int(pt_map[0]);
    int y = int(pt_map[1]);
    return Vec2i(x, y);
  }

  cv::Mat occupancy_grid_;

  std::vector<Model2Dpoint> model_;

  SE2 pose_;  // T_W_S
  Vec2d center_image_ = Vec2d(image_size_ / 2, image_size_ / 2);
  bool has_outside_pts_ = false;

  inline static constexpr int image_size_ = 1000;
  inline static constexpr int model_size_ = 400;
  inline static constexpr float inv_resolution_ = 0.05;
  inline static constexpr double resolution_ = 20.0;
  inline static constexpr double closest_th_ = 0.2;
  inline static constexpr double endpoint_close_th_ = 0.1;
};

}  // namespace zqx