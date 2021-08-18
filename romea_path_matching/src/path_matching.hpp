#ifndef __PathMatchingSystem_HPP__
#define __PathMatchingSystem_HPP__


//local
#include "path_matching_diagnostic.hpp"
#include "path_matching_display.hpp"

//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


//romea
#include <romea_localisation/PoseAndTwist3D.hpp>
#include <romea_path/PathMatching2D.hpp>
#include <romea_path/Path2D.hpp>
#include <romea_common_utils/publishers/diagnostic_publisher.hpp>

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

protected:

  romea::Path2D path_;
  romea::PathMatching2D path_matching_;
  romea::PathMatchedPoint2D::Opt matched_point_;
  double prediction_time_horizon_;
  std::string path_frame_id_;

  ros::Subscriber odom_sub_;
  ros::Publisher match_pub_;
  DiagnosticPublisher<DiagnosticReport> diagnostic_pub_;


  Eigen::Affine3d map_to_path_;
  Eigen::Affine3d world_to_path_;
  Eigen::Affine3d world_to_map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped tf_world_to_path_msg_;


  PathMatchingDisplay rviz_;
  PathMatchingDiagnostic diagnostics_;

};

}
#endif
