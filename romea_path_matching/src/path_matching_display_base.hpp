#ifndef PathMatchingDisplayBase_HPP
#define PathMatchingDisplayBase_HPP


//romea
#include <romea_path/PathSection2D.hpp>
#include <romea_path/PathWayPoint2D.hpp>
#include <romea_localisation_utils/rviz_display.hpp>

namespace romea {

class PathMatchingDisplayBase
{

public :

  PathMatchingDisplayBase();

  virtual ~PathMatchingDisplayBase()=default;

  void init(ros::NodeHandle private_nh);

  void displayPathMarkers(const Pose2D & vehicle_pose);

  void displayCurveMarkers(const PathCurve2D & path_curve);

  void deleteMarkers();

  void trigger();

  void reset();

protected:

  ros::Publisher match_pub_;

  rviz_visual_tools::RvizVisualTools rviz_util_;
#if ROS_VERSION_MINIMUM(1,12,0)
  romea::VectorOfEigenVector3d path3d_;
  romea::VectorOfEigenVector3d interpolatedPath3d_;
#else
  std::vector<Eigen::Vector3d> path3d_;
  std::vector<Eigen::Vector3d> interpolatedPath3d_;
#endif

  bool is_display_activated_;
};

}
#endif
