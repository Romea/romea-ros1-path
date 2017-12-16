#ifndef _romea_MatchedPointConversions_hpp_
#define _romea_MatchedPointConversions_hpp

//romea
#include <romea_path_msgs/PathMatchedPoint2DStamped.h>
#include "../PathMatchedPoint2D.hpp"
#include <ros/TimeConversions.hpp>

namespace romea {


romea_path_msgs::PathPosture2D toROSMsg(const PathPosture2D & posture);

romea_path_msgs::PathFrenetPose2D toROSMsg(const PathFrenetPose2D & frenet_pose);

romea_path_msgs::PathMatchedPoint2D toROSMsg(const PathMatchedPoint2D & matched_point);

romea_path_msgs::PathMatchedPoint2DStamped toROSMsg(const Duration & duration,const PathMatchedPoint2D matched_point);

romea_path_msgs::PathMatchedPoint2DStamped toROSMsg(const ros::Time & stamp,const PathMatchedPoint2D matched_point);


PathPosture2D toRomea(const romea_path_msgs::PathPosture2D &posture_msg);

PathFrenetPose2D toRomea(const romea_path_msgs::PathFrenetPose2D & frenet_pose_msg);

PathMatchedPoint2D toRomea(const romea_path_msgs::PathMatchedPoint2D & matched_point_msg);

PathMatchedPoint2D::Stamped toRomea(const romea_path_msgs::PathMatchedPoint2DStamped & stamped_matched_point_msg);

}

#endif
