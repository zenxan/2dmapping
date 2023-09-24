#pragma once

#include "2dmapping/frame.h"
#include "2dmapping/likelihood_field.h"
#include "2dmapping/occupancy_map.h"

namespace zqx {

class Submap {
 public:
  Submap(const SE2& pose) : pose_(pose) {
    field_.SetPose(pose);
    occu_map_.SetPose(pose);
  }

  bool MatchScan(std::shared_ptr<Frame> frame);

  void AddKeyFrame(std::shared_ptr<Frame> frame) { frames_.emplace_back(frame); }

  void AddScanInOccupancyMap(std::shared_ptr<Frame> frame);

  void SetOccuFromOtherSubmap(std::shared_ptr<Submap> other);

  OccupancyMap& GetOccuMap() { return occu_map_; }

  bool HasOutsidePoints() const;

  size_t NumFrames() const;
  std::vector<std::shared_ptr<Frame>>& GetFrames() { return frames_; }

  void SetId(size_t id) { id_ = id; }
  size_t GetId() const { return id_; }

  void SetPose(const SE2& pose);
  SE2 GetPose() const { return pose_; }

 private:
  SE2 pose_;  // T_W_S
  size_t id_ = 0;

  std::vector<std::shared_ptr<Frame>> frames_;
  LikelihoodField field_;
  OccupancyMap occu_map_;
};

}  // namespace zqx