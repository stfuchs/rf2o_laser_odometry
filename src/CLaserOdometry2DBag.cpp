#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_msgs/TFMessage.h>

#include "rf2o_laser_odometry/CLaserOdometry2D.h"


int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "Please provide path to bagfile." << std::endl;
    return EXIT_FAILURE;
  }

  const std::string bag_path = argv[1];
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  rf2o::CLaserOdometry2D lo;

  std::cout << "stamp,x,y,w,dx,dw" << std::endl;
  rosbag::View scan_view(bag, rosbag::TopicQuery(std::vector<std::string>{"/base_scan"}));
  for (const auto& m : scan_view)
  {
    auto scan = m.instantiate<sensor_msgs::LaserScan>();
    if (!scan) { continue; }

    if (!lo.is_initialized())
    {
      geometry_msgs::Pose pose;
      pose.orientation.w = 1.;
      //lo.setLaserPose()
      lo.init(*scan, pose);
      continue;
    }

    lo.odometryCalculation(*scan);
    double x = lo.getPose().translation()(0);
    double y = lo.getPose().translation()(1);
    double w = rf2o::getYaw(lo.getPose().rotation());
    double dx = lo.getLinearVelocity();
    double dw = lo.getAngularVelocity();
    std::cout << scan->header.stamp.toNSec() << "," << x << "," << y << "," << w << "," << dx << "," << dw << std::endl;

  }

  return EXIT_SUCCESS;
}
