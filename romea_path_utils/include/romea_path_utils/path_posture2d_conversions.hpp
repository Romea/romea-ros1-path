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

#ifndef _romea_PathPosture2DConversions_hpp_
#define _romea_PathPosture2DConversions_hpp_

//romea
#include <romea_core_path/PathPosture2D.hpp>
#include <romea_path_msgs/PathPosture2D.h>

namespace romea {

void to_ros_msg(const core::PathPosture2D & romea_path_posture2d,
              romea_path_msgs::PathPosture2D & ros_path_posture2d_msg);

void to_romea(const romea_path_msgs::PathPosture2D &posture_msg,
             core::PathPosture2D & romea_path_posture);

core::PathPosture2D to_romea(const romea_path_msgs::PathPosture2D &posture_msg);

}

#endif
