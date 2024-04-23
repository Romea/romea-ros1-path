#include "romea_path_utils/path_matching_point2d_conversions.hpp"
#include "romea_path_utils/path_frenet_pose2d_conversions.hpp"
#include "romea_path_utils/path_posture2d_conversions.hpp"

namespace romea {


//-----------------------------------------------------------------------------
void to_ros_msg(const core::PathMatchedPoint2D & romea_matched_point2d,
              romea_path_msgs::PathMatchedPoint2D & ros_path_matched_point2d_msg)
{
  to_ros_msg(romea_matched_point2d.pathPosture,ros_path_matched_point2d_msg.posture);
  to_ros_msg(romea_matched_point2d.frenetPose,ros_path_matched_point2d_msg.frenet_pose);
  ros_path_matched_point2d_msg.future_curvature= romea_matched_point2d.futureCurvature;
  ros_path_matched_point2d_msg.desired_speed= romea_matched_point2d.desiredSpeed;
  ros_path_matched_point2d_msg.section_index = romea_matched_point2d.sectionIndex;
  ros_path_matched_point2d_msg.section_minimal_curvilinear_abscissa =
    romea_matched_point2d.sectionMinimalCurvilinearAbscissa;
  ros_path_matched_point2d_msg.section_maximal_curvilinear_abscissa =
    romea_matched_point2d.sectionMaximalCurvilinearAbscissa;
  ros_path_matched_point2d_msg.curve_index = romea_matched_point2d.curveIndex;
}


//-----------------------------------------------------------------------------
void to_romea(const romea_path_msgs::PathMatchedPoint2D &matched_point_msg,
             core::PathMatchedPoint2D & romea_matched_point)
{
  to_romea(matched_point_msg.posture,romea_matched_point.pathPosture);
  to_romea(matched_point_msg.frenet_pose,romea_matched_point.frenetPose);
  romea_matched_point.desiredSpeed=matched_point_msg.desired_speed;
  romea_matched_point.sectionIndex=matched_point_msg.section_index;
  romea_matched_point.sectionMinimalCurvilinearAbscissa =
    matched_point_msg.section_minimal_curvilinear_abscissa;
  romea_matched_point.sectionMaximalCurvilinearAbscissa =
    matched_point_msg.section_maximal_curvilinear_abscissa;
  romea_matched_point.curveIndex=matched_point_msg.curve_index;
}


//-----------------------------------------------------------------------------
core::PathMatchedPoint2D to_romea(const romea_path_msgs::PathMatchedPoint2D &matched_point_msg)
{
  core::PathMatchedPoint2D romea_matched_point;
  to_romea(matched_point_msg,romea_matched_point);
  return romea_matched_point;
}

}
