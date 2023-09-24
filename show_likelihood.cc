#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/highgui.hpp>

#include "2dmapping/lidar_2d_utils.h"
#include "2dmapping/likelihood_field.h"
#include "common/io_utils.h"

DEFINE_string(bag_path, "./2dbag/floor1.bag", "");
DEFINE_string(method, "gauss-newton", "gauss-newton/g2o");

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
  Scan2d::Ptr last_scan = nullptr, current_scan = nullptr;

  rosbag_io
      .AddScan2DHandle("/pavo_scan_bottom",
                       [&](Scan2d::Ptr scan) {
                         zqx::LikelihoodField lf;
                         current_scan = scan;
                         SE2 pose;

                         if (last_scan == nullptr) {
                           last_scan = current_scan;
                           return true;
                         }

                         lf.SetTargetScan(last_scan);
                         lf.SetSourceScan(current_scan);

                         if (FLAGS_method == "gauss-newton") {
                           lf.AlignGaussNewton(pose);
                         }

                         LOG(INFO) << "aligned pose: " << pose.translation().transpose() << ", "
                                   << pose.so2().log();

                         cv::Mat image;
                         zqx::Visualize2DScan(last_scan, SE2(), image, Vec3b(255, 0, 0));
                         zqx::Visualize2DScan(current_scan, pose, image, Vec3b(0, 0, 255));
                         cv::imshow("scan", image);

                         cv::Mat field_image = lf.GetFieldImage();
                         zqx::Visualize2DScan(last_scan, SE2(), field_image, Vec3b(255, 0, 0), 1000,
                                              20.0);
                         cv::imshow("field", field_image);

                         cv::waitKey(10);

                         return true;
                       })
      .Go();
  return 0;
}