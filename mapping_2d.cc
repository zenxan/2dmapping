#include "mapping_2d.h"
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include "2dmapping/lidar_2d_utils.h"

namespace zqx {

bool Mapping2D::Init(bool with_loop_closing) {
  keyframe_id_ = 0;
  current_submap_ = std::make_shared<Submap>(SE2());
  all_submaps_.emplace_back(current_submap_);

  if (with_loop_closing) {
  }

  return true;
}

bool Mapping2D::ProcessScan(Scan2d::Ptr scan) {
  current_frame_ = std::make_shared<Frame>(scan);
  current_frame_->id_ = frame_id_++;

  if (last_frame_) {
    current_frame_->pose_ = last_frame_->pose_ * motion_guess_;
    current_frame_->pose_submap_ = last_frame_->pose_submap_;
  }

  if (!first_scan_) {
    current_submap_->MatchScan(current_frame_);  // Todo
  }

  first_scan_ = false;
  bool is_kf = IsKeyFrame();

  if (is_kf) {
    AddKeyFrame();
    current_submap_->AddScanInOccupancyMap(current_frame_);

    // if(loop_closing_){}

    if (current_submap_->HasOutsidePoints() || current_submap_->NumFrames() > 50) {
      ExpandSubmap();
    }
  }

  auto occu_image = current_submap_->GetOccuMap().GetOccupancyGridBlackWhite();
  Visualize2DScan(current_frame_->scan_, current_frame_->pose_, occu_image, Vec3b(0, 0, 255), 1000,
                  20.0, current_submap_->GetPose());
  cv::putText(occu_image, "submap " + std::to_string(current_submap_->GetId()), cv::Point2f(20, 20),
              cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
  cv::putText(occu_image, "keyframe " + std::to_string(current_submap_->NumFrames()),
              cv::Point2f(20, 50), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
  cv::imshow("occupancy map", occu_image);

  if (is_kf) std::cout << "pose " << current_frame_->pose_.translation().transpose() << std::endl;

  cv::waitKey(10);
  if (last_frame_) {
    motion_guess_ = last_frame_->pose_.inverse() * current_frame_->pose_;
  }

  last_frame_ = current_frame_;

  return true;
}

bool Mapping2D::IsKeyFrame() {
  if (last_frame_ == nullptr) {
    return true;
  }

  SE2 delta_pose = last_frame_->pose_.inverse() * current_frame_->pose_;
  if (delta_pose.translation().norm() > keyframe_pos_th_ ||
      fabs(delta_pose.so2().log()) > keyframe_ang_th_) {
    return true;
  }

  return false;
}

void Mapping2D::AddKeyFrame() {
  LOG(INFO) << "add key frame " << keyframe_id_;
  current_frame_->keyframe_id_ = keyframe_id_++;
  current_submap_->AddKeyFrame(current_frame_);
  last_keyframe_ = current_frame_;
}

void Mapping2D::ExpandSubmap() {
  // if(loop_closing){}
  auto last_submap = current_submap_;

  cv::imwrite("./data/zqx/submap_" + std::to_string(last_submap->GetId()) + ".png",
              last_submap->GetOccuMap().GetOccupancyGridBlackWhite());

  current_submap_ = std::make_shared<Submap>(current_frame_->pose_);
  current_frame_->pose_submap_ = SE2();

  current_submap_->SetId(++submap_id_);
  current_submap_->AddKeyFrame(current_frame_);
  current_submap_->SetOccuFromOtherSubmap(last_submap);
  current_submap_->AddScanInOccupancyMap(current_frame_);

  all_submaps_.emplace_back(current_submap_);

  // if(loop_closing){}

  LOG(INFO) << "create submap " << current_submap_->GetId()
            << ", with pose: " << current_submap_->GetPose().translation().transpose() << ", "
            << current_submap_->GetPose().so2().log();
}

}  // namespace zqx