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

#include "romea_path_utils/path_posture2d_conversions.hpp"

namespace romea {

//-----------------------------------------------------------------------------
void to_ros_msg(const core::PathPosture2D & romea_path_posture2d,
              romea_path_msgs::PathPosture2D & ros_path_posture2d_msg)
{
  ros_path_posture2d_msg.x = romea_path_posture2d.position.x();
  ros_path_posture2d_msg.y = romea_path_posture2d.position.y();
  ros_path_posture2d_msg.course = romea_path_posture2d.course;
  ros_path_posture2d_msg.curvature = romea_path_posture2d.curvature;
  ros_path_posture2d_msg.dot_curvature = romea_path_posture2d.dotCurvature;
}

//-----------------------------------------------------------------------------
void to_romea(const romea_path_msgs::PathPosture2D & posture_msg,
             core::PathPosture2D & romea_path_posture)
{
  romea_path_posture.position.x()=posture_msg.x;
  romea_path_posture.position.y()=posture_msg.y;
  romea_path_posture.course=posture_msg.course;
  romea_path_posture.curvature=posture_msg.curvature;
  romea_path_posture.dotCurvature=posture_msg.dot_curvature;
}

//-----------------------------------------------------------------------------
core::PathPosture2D to_romea(const romea_path_msgs::PathPosture2D &posture_msg)
{
  core::PathPosture2D romea_path_posture;
  to_romea(posture_msg,romea_path_posture);
  return romea_path_posture;
}

}
