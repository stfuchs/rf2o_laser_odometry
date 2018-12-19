#include <fstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>

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

  rosbag::View tf_view(bag, rosbag::TopicQuery(std::vector<std::string>{"/tf","/tf_static"}));
  // fill tf buffer first
  tf2_ros::Buffer buffer(tf_view.getEndTime()-tf_view.getBeginTime());

  for (const auto& m : tf_view)
  {
    auto tf_msg = m.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg)
    {
      for (const auto& tf : tf_msg->transforms)
      {
        buffer.setTransform(tf, "bag", m.getTopic() == "/tf_static");
      }
    }
  }


  rf2o::CLaserOdometry2D lo;
  lo.setVerbose(true);
  std::ofstream csv;
  csv.open("laser_odom.csv");
  csv << std::string("stamp,source,x,y,w,dx,dw") << std::endl;

  rosbag::View scan_view(bag, rosbag::TopicQuery(std::vector<std::string>{"/base_scan"}));
  for (const auto& m : scan_view)
  {
    auto scan = m.instantiate<sensor_msgs::LaserScan>();
    if (!scan) { continue; }

    if (!lo.is_initialized())
    {
      try
      {
        auto laser = rf2o::fromMsg(
          buffer.lookupTransform("base_link", scan->header.frame_id, scan->header.stamp).transform);
        auto robot = rf2o::fromMsg(
          buffer.lookupTransform("odom", "base_link", scan->header.stamp).transform);
        lo.setLaserPose(laser);
        lo.init(*scan, robot);
      }
      catch (const tf2::TransformException& e)
      {
        std::cerr << "Skipping scan. Failed to lookup transform" << e.what() << std::endl;
        continue;
      }
    }

    lo.odometryCalculation(*scan);
    double x = lo.getPose().translation()(0);
    double y = lo.getPose().translation()(1);
    double w = rf2o::getYaw(lo.getPose().rotation());
    double dx = lo.getLinearVelocity();
    double dw = lo.getAngularVelocity();
    csv << scan->header.stamp.toNSec() << ",laser," << x << "," << y << "," << w << "," << dx << "," << dw << std::endl;
  }
  csv.close();
  return EXIT_SUCCESS;
}
