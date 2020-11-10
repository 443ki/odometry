#include <ros/ros.h>
#include "odometry.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_node");
  Odometry odometry;
  ROS_INFO("Odometryr was initialized!");

  ros::spin();
  return 0;
}
