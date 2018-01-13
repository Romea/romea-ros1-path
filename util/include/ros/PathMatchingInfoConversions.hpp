#ifndef _romea_PathMatchingInfoConversions_hpp_
#define _romea_PathMatchingInfoConversions_hpp

//romea
#include <Twist2D.hpp>
#include "../PathMatchedPoint2D.hpp"
#include <romea_path_msgs/PathMatchingInfo2D.h>

#include <ros/TimeConversions.hpp>
#include <ros/Twist2DConversions.hpp>



namespace romea {


romea_path_msgs::PathPosture2D toROSMsg(const PathPosture2D & posture);

romea_path_msgs::PathFrenetPose2D toROSMsg(const PathFrenetPose2D & frenet_pose);

romea_path_msgs::PathMatchedPoint2D toROSMsg(const PathMatchedPoint2D & matched_point);

romea_path_msgs::PathMatchingInfo2D toROSMsg(const Duration & duration,
                                             const PathMatchedPoint2D matched_point,
                                             const double & future_curvature,
                                             const Twist2D & twist);

romea_path_msgs::PathMatchingInfo2D toROSMsg(const ros::Time & stamp,
                                             const PathMatchedPoint2D matched_point,
                                             const double & future_curvature,
                                             const Twist2D & twist);


PathPosture2D toRomea(const romea_path_msgs::PathPosture2D &posture_msg);

PathFrenetPose2D toRomea(const romea_path_msgs::PathFrenetPose2D & frenet_pose_msg);

PathMatchedPoint2D toRomea(const romea_path_msgs::PathMatchedPoint2D & matched_point_msg);


}

#endif
