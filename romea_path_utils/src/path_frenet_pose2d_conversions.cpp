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

#include "romea_path_utils/path_frenet_pose2d_conversions.hpp"

namespace romea {

//-----------------------------------------------------------------------------
void to_ros_msg(const core::PathFrenetPose2D & romea_frenet_pose2d,
              romea_path_msgs::PathFrenetPose2D & ros_frenet_pose2d_msg)
{
  ros_frenet_pose2d_msg.curvilinear_abscissa = romea_frenet_pose2d.curvilinearAbscissa;
  ros_frenet_pose2d_msg.lateral_deviation = romea_frenet_pose2d.lateralDeviation;
  ros_frenet_pose2d_msg.course_deviation = romea_frenet_pose2d.courseDeviation;

  for(size_t n=0;n<9;++n)
  {
    ros_frenet_pose2d_msg.covariance[n]=romea_frenet_pose2d.covariance(n);
  }
}

//-----------------------------------------------------------------------------
void to_romea(const romea_path_msgs::PathFrenetPose2D & frenet_pose_msg,
             core::PathFrenetPose2D & romea_frenet_pose)
{
  romea_frenet_pose.curvilinearAbscissa=frenet_pose_msg.curvilinear_abscissa;
  romea_frenet_pose.lateralDeviation=frenet_pose_msg.lateral_deviation;
  romea_frenet_pose.courseDeviation =frenet_pose_msg.course_deviation;
  romea_frenet_pose.covariance=Eigen::Matrix3d(frenet_pose_msg.covariance.data());
}

//-----------------------------------------------------------------------------
core::PathFrenetPose2D to_romea(const romea_path_msgs::PathFrenetPose2D &frenet_pose_msg)
{
  core::PathFrenetPose2D romea_frenet_pose;
  to_romea(frenet_pose_msg,romea_frenet_pose);
  return romea_frenet_pose;
}


}
