#ifndef __PathMatchingSystem_HPP__
#define __PathMatchingSystem_HPP__


//local
#include "path_matching_diagnostic.hpp"

//ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


//romea
#include <PoseAndTwist3D.hpp>
#include <PathMatching2D.hpp>
#include <Path2D.hpp>
#include <ros/RvizDisplay.hpp>
#include <romea_fsm_msgs/FSMService.h>

namespace romea {

class PathMatching
{

public :

  PathMatching();

  void init(ros::NodeHandle nh, ros::NodeHandle private_nh);

  void loadPath(const std::string & filename,bool revert);

  void publishTf(const ros::TimerEvent & event);

  void reset();

protected:


  void processOdom_(const nav_msgs::Odometry::ConstPtr &msg);

  bool tryToEvaluteMapToPathTransformation_(const ros::Time & stamp,
                                            const std::string & map_frame_id);

  bool tryToMatchOnPath_(const Pose2D & vehicle_pose);

  void initDisplay_();

  void displayInterpolatedPath_();

  void displayResults_(const Pose2D & vehicle_pose);

protected:

  romea::Path2D path_;
  romea::PathMatching2D path_matching_;
  romea::PathMatchedPoint2D::Opt matched_point_;
  double prediction_time_horizon_;

  ros::Subscriber odom_sub_;
  ros::Publisher match_pub_;

  Eigen::Affine3d map_to_path_;
  tf::Transform tf_world_to_path_;
  tf::StampedTransform tf_world_to_map_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped tf_world_to_path_msg_;


  bool display_;
  rviz_visual_tools::RvizVisualTools rviz_util_;
#if ROS_VERSION_MINIMUM(1,12,0)
  romea::VectorOfEigenVector3d path3d_;
  romea::VectorOfEigenVector3d interpolatedPath3d_;
#else
  std::vector<Eigen::Vector3d> path3d_;
  std::vector<Eigen::Vector3d> interpolatedPath3d_;
#endif

  PathMatchingDiagnostic diagnostics_;

};

}
#endif
