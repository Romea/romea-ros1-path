#ifndef _romea_PathMatchingInfoConversions_hpp_
#define _romea_PathMatchingInfoConversions_hpp

//romea
#include <Twist2D.hpp>
#include "../PathMatchedPoint2D.hpp"
#include <romea_path_msgs/PathMatchingInfo2D.h>

#include <ros/conversions/TimeConversions.hpp>
#include <ros/conversions/Twist2DConversions.hpp>



namespace romea {


void toRosMsg(const PathPosture2D & romea_path_posture2d,
              romea_path_msgs::PathPosture2D & ros_path_posture2d_msg);

void toRosMsg(const PathFrenetPose2D & romea_frenet_pose2d,
              romea_path_msgs::PathFrenetPose2D & ros_frenet_pose2d_msg);

void toRosMsg(const PathMatchedPoint2D & romea_matched_point2d,
              romea_path_msgs::PathMatchedPoint2D & ros_path_matched_point2d_msg);

romea_path_msgs::PathMatchingInfo2D toRosMsg(const Duration & duration,
                                             const PathMatchedPoint2D matched_point,
                                             const double &path_length,
                                             const double & future_curvature,
                                             const Twist2D & twist);

romea_path_msgs::PathMatchingInfo2D toRosMsg(const ros::Time & stamp,
                                             const PathMatchedPoint2D matched_point,
                                             const double & path_length,
                                             const double & future_curvature,
                                             const Twist2D & twist);


PathPosture2D toRomea(const romea_path_msgs::PathPosture2D &posture_msg);

PathFrenetPose2D toRomea(const romea_path_msgs::PathFrenetPose2D & frenet_pose_msg);

PathMatchedPoint2D toRomea(const romea_path_msgs::PathMatchedPoint2D & matched_point_msg);


}

#endif
