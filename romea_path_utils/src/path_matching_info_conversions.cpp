#include "romea_path_utils/path_matching_info_conversions.hpp"
#include "romea_path_utils/path_matching_point2d_conversions.hpp"
#include <romea_common_utils/conversions/time_conversions.hpp>
#include <romea_localisation_utils/conversions/twist2d_conversions.hpp>

namespace romea {

//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchingInfo2D toRosMsg(const Duration & duration,
                                             const PathMatchedPoint2D matched_point,
                                             const double & path_length,
                                             const double & future_curvature,
                                             const Twist2D & twist)
{
  return toRosMsg(toROSTime(duration),matched_point,path_length,future_curvature,twist);
}

//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchingInfo2D toRosMsg(const ros::Time & stamp,
                                             const PathMatchedPoint2D matched_point,
                                             const double & path_length,
                                             const double & future_curvature,
                                             const Twist2D & twist)
{
  romea_path_msgs::PathMatchingInfo2D msg;
  msg.header.frame_id="path";
  msg.header.stamp = stamp;
  toRosMsg(matched_point,msg.matched_point);
  msg.path_length =path_length;
  msg.future_curvature = future_curvature;
  toRosMsg(twist,msg.twist);
  return msg;

}

}
