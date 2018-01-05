#ifndef __WGS84PathMatchingSystem_HPP__
#define __WGS84PathMatchingSystem_HPP__


//local
#include "wgs84_path_matching_diagnostic.hpp"

//ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


//romea
#include <PoseAndTwist3D.hpp>
#include <PathMatching2D.hpp>
#include <WGS84Path2D.hpp>
#include <ros/Pose2DRvizDisplay.hpp>

class WGS84PathMatchingSystem
{

public :

  WGS84PathMatchingSystem(ros::NodeHandle node, ros::NodeHandle private_nh);

  void processOdom(const nav_msgs::Odometry::ConstPtr &msg);

protected:

  void loadPath_(const std::string & filename);

  void publishTf_(const ros::TimerEvent & event);

  //  void displayResults(const romea::Duration &duration,
  //                      const std::vector<romea::Range::Ptr> & ranges);

protected:

  romea::WGS84Path2D wgs84_path_;
  romea::PathMatching2D enu_path_matching_;
  romea::PathMatchedPoint2D::Opt enu_matched_point_;

  ros::Subscriber odom_sub_;
  ros::Publisher match_pub_;
  ros::Timer timer_;

  Eigen::Affine3d tf_map_to_path_;
  tf::Transform tf_world_to_path_;
  tf::StampedTransform tf_world_to_map_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped tf_world_to_path_msg_;


  bool display_;
  rviz_visual_tools::RvizVisualTools rviz_util_;
#if ROS_VERSION_MINIMUM(1,12,0)
  romea::VectorOfEigenVector<Eigen::Vector3d> path3d_;
  romea::VectorOfEigenVector<Eigen::Vector3d> interpolatedPath3d_;
#else
  std::vector<Eigen::Vector3d> path3d_;
  std::vector<Eigen::Vector3d> interpolatedPath3d_;
#endif

  WGS84PathMatchingDiagnostic diagnostics_;

};


#endif
