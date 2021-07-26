#ifndef _romea_PathFrenetPose2D_hpp_
#define _romea_PathFrenetPose2D_hpp

//romea
#include <romea_path/PathFrenetPose2D.hpp>
#include <romea_path_msgs/PathFrenetPose2D.h>

namespace romea {

void toRosMsg(const PathFrenetPose2D & romea_frenet_pose2d,
              romea_path_msgs::PathFrenetPose2D & ros_frenet_pose2d_msg);

void toRomea(const romea_path_msgs::PathFrenetPose2D & frenet_pose_msg,
             PathFrenetPose2D & romea_frenet_pose);

PathFrenetPose2D toRomea(const romea_path_msgs::PathFrenetPose2D &frenet_pose_msg);

}

#endif