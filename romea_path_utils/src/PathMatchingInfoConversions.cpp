#include "romea_path_utils/PathMatchingInfoConversions.hpp"


namespace romea {


//-----------------------------------------------------------------------------
void toRosMsg(const PathPosture2D & romea_path_posture2d,
              romea_path_msgs::PathPosture2D & ros_path_posture2d_msg)
{
  ros_path_posture2d_msg.x = romea_path_posture2d.position.x();
  ros_path_posture2d_msg.y = romea_path_posture2d.position.y();
  ros_path_posture2d_msg.course = romea_path_posture2d.course;
  ros_path_posture2d_msg.curvature = romea_path_posture2d.curvature;
  ros_path_posture2d_msg.dot_curvature = romea_path_posture2d.dotCurvature;
}

//-----------------------------------------------------------------------------
void toRosMsg(const PathFrenetPose2D & romea_frenet_pose2d,
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
void toRosMsg(const PathMatchedPoint2D & romea_matched_point2d,
              romea_path_msgs::PathMatchedPoint2D & ros_path_matched_point2d_msg)
{
  toRosMsg(romea_matched_point2d.pathPosture,ros_path_matched_point2d_msg.posture);
  toRosMsg(romea_matched_point2d.frenetPose,ros_path_matched_point2d_msg.frenet_pose);
  ros_path_matched_point2d_msg.nearest_point_index = romea_matched_point2d.nearestPointIndex;
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
void toRomea(const romea_path_msgs::PathPosture2D & posture_msg,
             PathPosture2D & romea_path_posture)
{
  romea_path_posture.position.x()=posture_msg.x;
  romea_path_posture.position.y()=posture_msg.y;
  romea_path_posture.course=posture_msg.course;
  romea_path_posture.curvature=posture_msg.curvature;
  romea_path_posture.dotCurvature=posture_msg.dot_curvature;
}

//-----------------------------------------------------------------------------
PathPosture2D toRomea(const romea_path_msgs::PathPosture2D &posture_msg)
{
  PathPosture2D romea_path_posture;
  toRomea(posture_msg,romea_path_posture);
  return romea_path_posture;
}

//-----------------------------------------------------------------------------
void toRomea(const romea_path_msgs::PathFrenetPose2D & frenet_pose_msg,
             PathFrenetPose2D & romea_frenet_pose)
{
  romea_frenet_pose.curvilinearAbscissa=frenet_pose_msg.curvilinear_abscissa;
  romea_frenet_pose.lateralDeviation=frenet_pose_msg.lateral_deviation;
  romea_frenet_pose.courseDeviation =frenet_pose_msg.course_deviation;
  romea_frenet_pose.covariance=Eigen::Matrix3d(frenet_pose_msg.covariance.data());
}

//-----------------------------------------------------------------------------
PathFrenetPose2D toRomea(const romea_path_msgs::PathFrenetPose2D &frenet_pose_msg)
{
  PathFrenetPose2D romea_frenet_pose;
  toRomea(frenet_pose_msg,romea_frenet_pose);
  return romea_frenet_pose;
}

//-----------------------------------------------------------------------------
void toRomea(const romea_path_msgs::PathMatchedPoint2D &matched_point_msg,
             PathMatchedPoint2D & romea_matched_point)
{
  toRomea(matched_point_msg.posture,romea_matched_point.pathPosture);
  toRomea(matched_point_msg.frenet_pose,romea_matched_point.frenetPose);
  romea_matched_point.nearestPointIndex=matched_point_msg.nearest_point_index;
}

//-----------------------------------------------------------------------------
PathMatchedPoint2D toRomea(const romea_path_msgs::PathMatchedPoint2D &matched_point_msg)
{
  PathMatchedPoint2D romea_matched_point;
  toRomea(matched_point_msg,romea_matched_point);
  return romea_matched_point;
}


}
