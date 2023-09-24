#pragma once

#include "common/eigen_types.h"
#include "common/lidar_utils.h"

namespace zqx {

struct Frame {
  Frame() {}
  Frame(Scan2d::Ptr scan) : scan_(scan) {}

  size_t id_ = 0;
  size_t keyframe_id_ = 0;
  double timestamp_ = 0;
  Scan2d::Ptr scan_ = nullptr;
  SE2 pose_;         // T_W_C
  SE2 pose_submap_;  // T_S_C
};

}  // namespace zqx
