#include <ros/ros.h>
#include "wgs84_path_matching_system.hpp"
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_to_human_localisation");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  WGS84PathMatchingSystem path(node, private_nh);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  //ros::spin();

}
