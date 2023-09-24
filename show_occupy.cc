#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/highgui.hpp>

#include "common/io_utils.h"
#include "common/sys_utils.h"
#include "2dmapping/lidar_2d_utils.h"
#include "2dmapping/occupancy_map.h"

DEFINE_string(bag_path, "./2dbag/floor1.bag", "");
DEFINE_string(method, "0", "0 or 1");

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);

  rosbag_io
      .AddScan2DHandle("/pavo_scan_bottom",
                       [&](Scan2d::Ptr scan) {
                         zqx::OccupancyMap oc_map;
                         sad::evaluate_and_call(
                             [&]() {
                               oc_map.AddLidarFrame(std::make_shared<zqx::Frame>(scan),
                                                    zqx::OccupancyMap::GridMethod::MODEL_POINTS);
                             },
                             "Occupancy of mine");

                        // cv::imshow("occupancy map", oc_map.GetOccupancyGridBlackWhite());
                        // cv::waitKey(10);
                         return true;
                       })
      .Go();

  return 0;
}