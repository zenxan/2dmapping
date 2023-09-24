#pragma once

#include <opencv2/core.hpp>
#include "common/eigen_types.h"
#include "common/lidar_utils.h"

namespace zqx {

class LikelihoodField {
 public:
  struct ModelPoint {
    ModelPoint(int dx, int dy, float res) : dx_(dx), dy_(dy), residual_(res) {}
    int dx_ = 0;
    int dy_ = 0;
    float residual_ = 0;
  };

  LikelihoodField() { BuildModel(); }

  void SetTargetScan(Scan2d::Ptr scan);

  void SetSourceScan(Scan2d::Ptr scan);

  bool AlignGaussNewton(SE2& init_pose);

  cv::Mat GetFieldImage();

  void SetFieldImageFromOccuMap(const cv::Mat& occu_map);

  void SetPose(const SE2& pose) { pose_ = pose; }

 private:
  void BuildModel();

  SE2 pose_;
  Scan2d::Ptr target_ = nullptr;
  Scan2d::Ptr source_ = nullptr;

  std::vector<ModelPoint> model_;
  cv::Mat field_;
  bool has_outside_pts_ = false;

  inline static const float resolution_ = 20;
};

}  // namespace zqx