#pragma once

#include "2dmapping/frame.h"
#include "2dmapping/submap.h"
#include "common/eigen_types.h"
#include "common/lidar_utils.h"

#include <memory>
#include <opencv2/core.hpp>

namespace zqx {

class Mapping2D {
 public:
  bool Init(bool with_loop_closing = true);

  bool ProcessScan(Scan2d::Ptr scan);

  bool IsKeyFrame();

  void AddKeyFrame();

  void ExpandSubmap();

 private:
  size_t frame_id_ = 0;
  size_t keyframe_id_ = 0;
  size_t submap_id_ = 0;

  bool first_scan_ = true;

  SE2 motion_guess_;
  std::shared_ptr<Frame> current_frame_ = nullptr;
  std::shared_ptr<Frame> last_frame_ = nullptr;
  std::shared_ptr<Frame> last_keyframe_ = nullptr;
  std::shared_ptr<Submap> current_submap_ = nullptr;

  std::vector<std::shared_ptr<Submap>> all_submaps_;

  inline static constexpr double keyframe_pos_th_ = 0.3;
  inline static constexpr double keyframe_ang_th_ = 15 * M_PI / 180;
};

}  // namespace zqx