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

#include "romea_path_utils/path_builder.hpp"

#include <algorithm>

namespace romea
{
namespace ros1
{

core::Path2D create_path(const nav_msgs::Path & msg, double desired_speed, double interp_windows)
{
  if (msg.poses.size() < 2) {
    throw std::runtime_error("Cannot create a path with less than 2 points");
  }

  auto pose_it = std::cbegin(msg.poses);
  auto next_pose_it = pose_it + 1;
  auto pose_end = std::cend(msg.poses);
  bool prev_is_forward = check_forward(*pose_it, *next_pose_it);

  core::Path2D::WayPoints waypoints;
  waypoints.emplace_back();
  auto section_it = begin(waypoints);

  while (next_pose_it != pose_end) {
    // if the direction of the movement change, create a new section
    bool is_forward = check_forward(*pose_it, *next_pose_it);
    if (is_forward != prev_is_forward) {
      waypoints.emplace_back();
      section_it = end(waypoints) - 1;
    }

    push_waypoint(*section_it, *pose_it, (is_forward ? desired_speed : -desired_speed));

    ++pose_it;
    ++next_pose_it;
    prev_is_forward = is_forward;
  }

  push_waypoint(*section_it, *pose_it, desired_speed);

  // remove all the sections that contain less than 3 points
  waypoints.erase(
    std::remove_if(begin(waypoints), end(waypoints), [](const auto & e) { return e.size() < 3; }),
    end(waypoints));

  return core::Path2D(waypoints, interp_windows);
}

Eigen::Vector2d direction_from_quat(const geometry_msgs::Quaternion & quat)
{
  Eigen::Quaterniond q{quat.w, quat.x, quat.y, quat.z};
  return (q * Eigen::Vector3d{1, 0, 0}).head<2>();
}

Eigen::Vector2d direction_from_points(
  const geometry_msgs::PoseStamped & a, const geometry_msgs::PoseStamped & b)
{
  const auto & a_pos = a.pose.position;
  const auto & b_pos = b.pose.position;
  Eigen::Vector2d diff = Eigen::Vector2d{b_pos.x, b_pos.y} - Eigen::Vector2d{a_pos.x, a_pos.y};
  diff.normalize();
  return diff;
}

bool is_same_direction(Eigen::Vector2d a, Eigen::Vector2d b)
{
  return a.transpose() * b > 0.;
}

bool check_forward(const geometry_msgs::PoseStamped & a, const geometry_msgs::PoseStamped & b)
{
  const auto & a_quat = a.pose.orientation;
  auto quat_dir = direction_from_quat(a_quat);
  auto diff_dir = direction_from_points(a, b);
  return is_same_direction(quat_dir, diff_dir);
}

void push_waypoint(
  std::vector<core::PathWayPoint2D> & waypoints,
  const geometry_msgs::PoseStamped & pose,
  double speed)
{
  const auto & pos = pose.pose.position;
  Eigen::Vector2d point{pos.x, pos.y};
  waypoints.emplace_back(point, speed);
}

}  // namespace ros1
}  // namespace romea
