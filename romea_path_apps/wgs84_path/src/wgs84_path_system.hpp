#ifndef __WGS84PathSystem_HPP__
#define __WGS84PathSystem_HPP__


//local
#include "wgs84_path_diagnostic.hpp"

//ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


//romea
#include <ENUPoseAndBodyTwist3D.hpp>
#include <ENUPathMatching2D.hpp>
#include <WGS84Path2D.hpp>
#include <ros_visual_util.hpp>
//#include <ros_diagnostic_util.hpp>


class WGS84PathSystem
{

public :

  WGS84PathSystem(ros::NodeHandle node, ros::NodeHandle private_nh);

  void processOdom(const nav_msgs::Odometry::ConstPtr &msg);

protected:

  void loadPath_(const std::string & path_filename);

  void publishTf_(const ros::TimerEvent & event);

//  void displayResults(const romea::Duration &duration,
//                      const std::vector<romea::Range::Ptr> & ranges);

protected:

  romea::WGS84Path2D wgs84_path_;
  romea::ENUPathMatching2D enu_path_matching_;
  romea::ENUPathMatchedPoint2D::Opt enu_matched_point_;

  ros::Subscriber odom_sub_;
  ros::Publisher match_pub_;
  ros::Timer timer_;

  tf::Transform tf_map_to_path_;
  tf::Transform tf_world_to_path_;
  tf::StampedTransform tf_world_to_map_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped tf_world_to_path_msg_;


  bool display_;
  rviz_visual_tools::RvizVisualTools rviz_util_;


//  WGS84PathDiagnostic diagnostics_;

};


#endif
