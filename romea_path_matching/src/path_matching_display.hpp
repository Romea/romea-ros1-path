#ifndef __PathMatchingDisplay_HPP__
#define __PathMatchingDisplay_HPP__



//ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


//romea
#include <romea_localisation/PoseAndTwist3D.hpp>
#include <romea_path/PathMatching2D.hpp>
#include <romea_path/Path2D.hpp>
#include <romea_common_utils/publishers/diagnostic_publisher.hpp>
#include <romea_localisation_utils/rviz_display.hpp>

namespace romea {

class PathMatchingDisplay
{

public :

  PathMatchingDisplay();

  void configure(const Path2D & path);

  void display(const Pose2D & vehicle_pose,
               const PathCurve2D * pathCurve);

  bool enable();

  bool disable();

private :

  void displayInterpolatedPath_(const PathCurve2D &pathCurve);

private :

  bool enable_;
  rviz_visual_tools::RvizVisualTools rviz_util_;

#if ROS_VERSION_MINIMUM(1,12,0)
  romea::VectorOfEigenVector3d path3d_;
  romea::VectorOfEigenVector3d interpolatedPath3d_;
#else
  std::vector<Eigen::Vector3d> path3d_;
  std::vector<Eigen::Vector3d> interpolatedPath3d_;
#endif

};

}
#endif
