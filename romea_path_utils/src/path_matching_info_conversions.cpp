#include "romea_path_utils/path_matching_info_conversions.hpp"

#include <romea_common_utils/conversions/time_conversions.hpp>
#include <romea_common_utils/conversions/twist2d_conversions.hpp>

#include "romea_path_utils/path_matching_point2d_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchingInfo2D to_ros_msg(
  const core::Duration & duration,
  const core::PathMatchedPoint2D & matched_point,
  const double & path_length,
  const core::Twist2D & twist)
{
  return to_ros_msg(to_ros_time(duration), matched_point, path_length, twist);
}

//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchingInfo2D to_ros_msg(
  const ros::Time & stamp,
  const core::PathMatchedPoint2D & matched_point,
  const double & path_length,
  const core::Twist2D & twist)
{
  romea_path_msgs::PathMatchingInfo2D msg;
  msg.header.frame_id = "path";
  msg.header.stamp = stamp;
  msg.matched_points.resize(1);
  to_ros_msg(matched_point, msg.matched_points[0]);
  msg.path_length = path_length;
  to_ros_msg(twist, msg.twist);
  return msg;
}

//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchingInfo2D to_ros_msg(
  const core::Duration & duration,
  const std::vector<core::PathMatchedPoint2D> & matched_points,
  const size_t & tracked_matched_point_index,
  const double & path_length,
  const core::Twist2D & twist)
{
  return to_ros_msg(
    to_ros_time(duration), matched_points, tracked_matched_point_index, path_length, twist);
}

//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchingInfo2D to_ros_msg(
  const ros::Time & stamp,
  const std::vector<core::PathMatchedPoint2D> & matched_points,
  const size_t & tracked_matched_point_index,
  const double & path_length,
  const core::Twist2D & twist)
{
  romea_path_msgs::PathMatchingInfo2D msg;
  msg.header.frame_id = "path";
  msg.header.stamp = stamp;
  msg.matched_points.resize(matched_points.size());
  for (size_t n = 0; n < matched_points.size(); ++n) {
    to_ros_msg(matched_points[n], msg.matched_points[n]);
  }
  msg.tracked_matched_point_index = tracked_matched_point_index;
  msg.path_length = path_length;
  to_ros_msg(twist, msg.twist);
  return msg;
}

//-----------------------------------------------------------------------------
std::vector<core::PathMatchedPoint2D> to_romea(
  const romea_path_msgs::PathMatchingInfo2D::_matched_points_type & matched_point_msgs)
{
  std::vector<core::PathMatchedPoint2D> romea_matched_points;
  for (const auto & msg : matched_point_msgs) {
    romea_matched_points.push_back(to_romea(msg));
  }
  return romea_matched_points;
}


}  // namespace romea
