// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

#ifndef ROMEA_PATH_UTILS__PATH_BUILDER_HPP_
#define ROMEA_PATH_UTILS__PATH_BUILDER_HPP_

// ros
#include <nav_msgs/Path.h>

// romea
#include <romea_core_path/Path2D.hpp>

namespace romea
{
namespace ros1
{

core::Path2D create_path(const nav_msgs::Path & msg, double desired_speed, double interp_window);

Eigen::Vector2d direction_from_quat(const geometry_msgs::Quaternion & quat);

Eigen::Vector2d direction_from_points(
  const geometry_msgs::PoseStamped & a, const geometry_msgs::PoseStamped & b);

bool is_same_direction(Eigen::Vector2d a, Eigen::Vector2d b);

bool check_forward(const geometry_msgs::PoseStamped & a, const geometry_msgs::PoseStamped & b);

void push_waypoint(
  std::vector<core::PathWayPoint2D> & waypoints,
  const geometry_msgs::PoseStamped & pose,
  double speed);

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_PATH_UTILS__PATH_UTILS_HPP_
