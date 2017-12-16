#include "ros/MatchedPointConversions.hpp"


namespace romea {

//-----------------------------------------------------------------------------
romea_path_msgs::PathPosture2D toROSMsg(const PathPosture2D & posture)
{
  romea_path_msgs::PathPosture2D msg;
  msg.x = posture.getX();
  msg.y = posture.getY();
  msg.course = posture.getCourse();
  msg.curvature = posture.getCurvature();
  msg.dot_curvature = posture.getDotCurvature();
  return msg;
}

//-----------------------------------------------------------------------------
romea_path_msgs::PathFrenetPose2D toROSMsg(const PathFrenetPose2D & frenet_pose)
{
  romea_path_msgs::PathFrenetPose2D msg;
  msg.curvilinear_abscissa = frenet_pose.getCurvilinearAbscissa();
  msg.lateral_deviation = frenet_pose.getLateralDeviation();
  msg.course_deviation = frenet_pose.getCourseDeviation();

  std::copy(frenet_pose.getCovariance().data(),
            frenet_pose.getCovariance().data()+9,
            msg.covariance.data());

  return msg;
}

//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchedPoint2D toROSMsg(const PathMatchedPoint2D & matched_point)
{
  romea_path_msgs::PathMatchedPoint2D msg;
  msg.posture = toROSMsg(matched_point.getPosture());
  msg.frenet_pose = toROSMsg(matched_point.getFrenetPose());
  msg.nearest_point_index = matched_point.getNearestPointIndex();
  return msg;
}

//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchedPoint2DStamped toROSMsg(const Duration & duration,const PathMatchedPoint2D matched_point)
{
  return toROSMsg(toROSTime(duration),matched_point);
}

//-----------------------------------------------------------------------------
romea_path_msgs::PathMatchedPoint2DStamped toROSMsg(const ros::Time & stamp,const PathMatchedPoint2D matched_point)
{
  romea_path_msgs::PathMatchedPoint2DStamped msg;
  msg.header.frame_id="matched_point";
  msg.header.stamp = stamp;
  msg.matched_point = toROSMsg(matched_point);
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

//-----------------------------------------------------------------------------
PathMatchedPoint2D::Stamped toRomea(const romea_path_msgs::PathMatchedPoint2DStamped &stamped_matched_point_msg)
{
  return PathMatchedPoint2D::Stamped(toRomeaDuration(stamped_matched_point_msg.header.stamp),
                                     toRomea(stamped_matched_point_msg.matched_point));
}

}
