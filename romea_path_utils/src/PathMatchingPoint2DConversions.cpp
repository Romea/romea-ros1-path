#include "romea_path_utils/PathMatchingPoint2DConversions.hpp"
#include "romea_path_utils/PathFrenetPose2DConversions.hpp"
#include "romea_path_utils/PathPosture2DConversions.hpp"

namespace romea {

//-----------------------------------------------------------------------------
void toRosMsg(const PathMatchedPoint2D & romea_matched_point2d,
              romea_path_msgs::PathMatchedPoint2D & ros_path_matched_point2d_msg)
{
  toRosMsg(romea_matched_point2d.pathPosture,ros_path_matched_point2d_msg.posture);
  toRosMsg(romea_matched_point2d.frenetPose,ros_path_matched_point2d_msg.frenet_pose);
  ros_path_matched_point2d_msg.nearest_point_index = romea_matched_point2d.nearestPointIndex;
}

//-----------------------------------------------------------------------------
void toRomea(const romea_path_msgs::PathMatchedPoint2D &matched_point_msg,
             PathMatchedPoint2D & romea_matched_point)
{
  toRomea(matched_point_msg.posture,romea_matched_point.pathPosture);
  toRomea(matched_point_msg.frenet_pose,romea_matched_point.frenetPose);
  romea_matched_point.nearestPointIndex=matched_point_msg.nearest_point_index;
}

//-----------------------------------------------------------------------------
PathMatchedPoint2D toRomea(const romea_path_msgs::PathMatchedPoint2D &matched_point_msg)
{
  PathMatchedPoint2D romea_matched_point;
  toRomea(matched_point_msg,romea_matched_point);
  return romea_matched_point;
}

}
