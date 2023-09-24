#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "2dmapping/lidar_2d_utils.h"
#include "2dmapping/mapping_2d.h"
#include "common/io_utils.h"

DEFINE_string(bag_path, "./2dbag/floor1.bag", "");
DEFINE_bool(with_loop_closing, false, "");

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
  zqx::Mapping2D mapping;

  if (mapping.Init(FLAGS_with_loop_closing) == false) {
    return -1;
  }

  rosbag_io
      .AddScan2DHandle("/pavo_scan_bottom",
                       [&](Scan2d::Ptr scan) {
                         static size_t num = 0;
                         num++;
                         if (num > 1400) {
                           mapping.ProcessScan(scan);
                         }
                         return true;
                       })
      .Go();

  return 0;
}