#include "romea_path_utils/PathPosture2DConversions.hpp"

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

}