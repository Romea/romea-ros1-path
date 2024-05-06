// Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _romea_PathMatchingInfoConversions_hpp_
#define _romea_PathMatchingInfoConversions_hpp

//romea
#include <romea_path_msgs/PathMatchingInfo2D.h>

#include <romea_core_common/geometry/Twist2D.hpp>
#include <romea_core_common/time/Time.hpp>
#include <romea_core_path/PathMatchedPoint2D.hpp>

namespace romea
{

//romea_path_msgs::PathMatchingInfo2D to_ros_msg(const Duration & duration,
//                                             const std::string &path_frame_id,
//                                             const PathMatchedPoint2D matched_point,
//                                             const double &path_length,
//                                             const double & future_curvature,
//                                             const Twist2D & twist);

//romea_path_msgs::PathMatchingInfo2D to_ros_msg(const ros::Time & stamp,
//                                             const std::string &path_frame_id,
//                                             const PathMatchedPoint2D matched_point,
//                                             const double & path_length,
//                                             const double & future_curvature,
//                                             const Twist2D & twist);

romea_path_msgs::PathMatchingInfo2D to_ros_msg(
  const core::Duration & duration,
  const core::PathMatchedPoint2D & matched_point,
  const double & path_length,
  const core::Twist2D & twist);

romea_path_msgs::PathMatchingInfo2D to_ros_msg(
  const ros::Time & stamp,
  const core::PathMatchedPoint2D & matched_point,
  const double & path_length,
  const core::Twist2D & twist);

romea_path_msgs::PathMatchingInfo2D to_ros_msg(
  const core::Duration & duration,
  const std::vector<core::PathMatchedPoint2D> & matched_points,
  const size_t & tracked_matched_point_index,
  const double & path_length,
  const core::Twist2D & twist);

romea_path_msgs::PathMatchingInfo2D to_ros_msg(
  const ros::Time & stamp,
  const std::vector<core::PathMatchedPoint2D> & matched_points,
  const size_t & tracked_matched_point_index,
  const double & path_length,
  const core::Twist2D & twist);

std::vector<core::PathMatchedPoint2D> to_romea(
  const romea_path_msgs::PathMatchingInfo2D::_matched_points_type & matched_point_msgs);

}  // namespace romea

#endif
