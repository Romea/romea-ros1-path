#include "ros/PathMatchingInfoConversions.hpp"


namespace romea {


//-----------------------------------------------------------------------------
void toRosMsg(const PathPosture2D & romea_path_posture2d,
              romea_path_msgs::PathPosture2D & ros_path_posture2d_msg)
{
  ros_path_posture2d_msg.x = romea_path_posture2d.getX();
  ros_path_posture2d_msg.y = romea_path_posture2d.getY();
  ros_path_posture2d_msg.course = romea_path_posture2d.getCourse();
  ros_path_posture2d_msg.curvature = romea_path_posture2d.getCurvature();
  ros_path_posture2d_msg.dot_curvature = romea_path_posture2d.getDotCurvature();
}

//-----------------------------------------------------------------------------
void toRosMsg(const PathFrenetPose2D & romea_frenet_pose2d,
              romea_path_msgs::PathFrenetPose2D & ros_frenet_pose2d_msg)
{
  ros_frenet_pose2d_msg.curvilinear_abscissa = romea_frenet_pose2d.getCurvilinearAbscissa();
  ros_frenet_pose2d_msg.lateral_deviation = romea_frenet_pose2d.getLateralDeviation();
  ros_frenet_pose2d_msg.course_deviation = romea_frenet_pose2d.getCourseDeviation();

  //  std::copy(romea_frenet_pose2d.getCovariance().data(),
  //            romea_frenet_pose2d.getCovariance().data()+9,
  //            ros_frenet_pose2d_msg.covariance.data());

}

//-----------------------------------------------------------------------------
void toRosMsg(const PathMatchedPoint2D & romea_matched_point2d,
              romea_path_msgs::PathMatchedPoint2D & ros_path_matched_point2d_msg)
{
  toRosMsg(romea_matched_point2d.getPosture(),ros_path_matched_point2d_msg.posture);
  toRosMsg(romea_matched_point2d.getFrenetPose(),ros_path_matched_point2d_msg.frenet_pose);
  ros_path_matched_point2d_msg.nearest_point_index = romea_matched_point2d.getNearestPointIndex();
}


//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchingInfo2D toRosMsg(const Duration & duration,
                                             const PathMatchedPoint2D matched_point,
                                             const double & path_length,
                                             const double & future_curvature,
                                             const Twist2D & twist)
{
  return toRosMsg(toROSTime(duration),matched_point,path_length,future_curvature,twist);
}

//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchingInfo2D toRosMsg(const ros::Time & stamp,
                                             const PathMatchedPoint2D matched_point,
                                             const double & path_length,
                                             const double & future_curvature,
                                             const Twist2D & twist)
{
  romea_path_msgs::PathMatchingInfo2D msg;
  msg.header.frame_id="path";
  msg.header.stamp = stamp;
  toRosMsg(matched_point,msg.matched_point);
  msg.path_length =path_length;
  msg.future_curvature = future_curvature;
  toRosMsg(twist,msg.twist);
  return msg;

}


//-----------------------------------------------------------------------------
PathPosture2D toRomea(const romea_path_msgs::PathPosture2D & posture_msg)
{
  return PathPosture2D(posture_msg.x,
                       posture_msg.y,
                       posture_msg.course,
                       posture_msg.curvature,
                       posture_msg.dot_curvature);
}

//-----------------------------------------------------------------------------
PathFrenetPose2D toRomea(const romea_path_msgs::PathFrenetPose2D & frenet_pose_msg)
{
  return PathFrenetPose2D(frenet_pose_msg.curvilinear_abscissa,
                          frenet_pose_msg.lateral_deviation,
                          frenet_pose_msg.course_deviation,
                          Eigen::Matrix3d(frenet_pose_msg.covariance.data()));
}

//-----------------------------------------------------------------------------
PathMatchedPoint2D toRomea(const romea_path_msgs::PathMatchedPoint2D &matched_point_msg)
{
  return PathMatchedPoint2D(toRomea(matched_point_msg.posture),
                            toRomea(matched_point_msg.frenet_pose),
                            matched_point_msg.nearest_point_index);
}


}
