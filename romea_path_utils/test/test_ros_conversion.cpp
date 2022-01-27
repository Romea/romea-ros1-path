// gtest
#include <gtest/gtest.h>

//romea
#include "romea_path_utils/path_matching_info_conversions.hpp"
#include "romea_path_utils/path_matching_point2d_conversions.hpp"
#include "romea_path_utils/path_frenet_pose2d_conversions.hpp"
#include "romea_path_utils/path_posture2d_conversions.hpp"


//-----------------------------------------------------------------------------
TEST(TestPosture2dConvertion, fromRosMsgto_romea )
{
  romea_path_msgs::PathPosture2D ros_posture2d_msg;
  ros_posture2d_msg.x=1;
  ros_posture2d_msg.y=2;
  ros_posture2d_msg.course=3;
  ros_posture2d_msg.curvature=4;
  ros_posture2d_msg.dot_curvature=5;

  romea::PathPosture2D romea_path_posture2d=romea::to_romea(ros_posture2d_msg);

  EXPECT_DOUBLE_EQ(romea_path_posture2d.position.x(),ros_posture2d_msg.x);
  EXPECT_DOUBLE_EQ(romea_path_posture2d.position.y(),ros_posture2d_msg.y);
  EXPECT_DOUBLE_EQ(romea_path_posture2d.course,ros_posture2d_msg.course);
  EXPECT_DOUBLE_EQ(romea_path_posture2d.curvature,ros_posture2d_msg.curvature);
  EXPECT_DOUBLE_EQ(romea_path_posture2d.dotCurvature,ros_posture2d_msg.dot_curvature);
}

//-----------------------------------------------------------------------------
TEST(TestPosture2dConvertion, fromRomeaToRosMsg )
{
  romea::PathPosture2D romea_path_posture2d;
  romea_path_posture2d.position.x()=1;
  romea_path_posture2d.position.y()=2;
  romea_path_posture2d.course=3;
  romea_path_posture2d.curvature=4;
  romea_path_posture2d.dotCurvature=5;

  romea_path_msgs::PathPosture2D ros_posture2d_msg;
  romea::to_ros_msg(romea_path_posture2d,ros_posture2d_msg);

  EXPECT_DOUBLE_EQ(romea_path_posture2d.position.x(),ros_posture2d_msg.x);
  EXPECT_DOUBLE_EQ(romea_path_posture2d.position.y(),ros_posture2d_msg.y);
  EXPECT_DOUBLE_EQ(romea_path_posture2d.course,ros_posture2d_msg.course);
  EXPECT_DOUBLE_EQ(romea_path_posture2d.curvature,ros_posture2d_msg.curvature);
  EXPECT_DOUBLE_EQ(romea_path_posture2d.dotCurvature,ros_posture2d_msg.dot_curvature);
}

//-----------------------------------------------------------------------------
TEST(TestFrenetPoseConvertion, fromRosMsgto_romea )
{
  romea_path_msgs::PathFrenetPose2D ros_frenet_pose2d_msg;
  ros_frenet_pose2d_msg.curvilinear_abscissa=1;
  ros_frenet_pose2d_msg.lateral_deviation=2;
  ros_frenet_pose2d_msg.course_deviation=3;
  for(size_t n=0;n<9;++n)
  {
    ros_frenet_pose2d_msg.covariance[n]=n;
  }

  romea::PathFrenetPose2D romea_frenet_pose2d=romea::to_romea(ros_frenet_pose2d_msg);

  EXPECT_DOUBLE_EQ(romea_frenet_pose2d.curvilinearAbscissa,ros_frenet_pose2d_msg.curvilinear_abscissa);
  EXPECT_DOUBLE_EQ(romea_frenet_pose2d.lateralDeviation,ros_frenet_pose2d_msg.lateral_deviation);
  EXPECT_DOUBLE_EQ(romea_frenet_pose2d.courseDeviation,ros_frenet_pose2d_msg.course_deviation);
  for(size_t n=0;n<9;++n)
  {
    EXPECT_DOUBLE_EQ(romea_frenet_pose2d.covariance(n),ros_frenet_pose2d_msg.covariance[n]);
  }
}

//-----------------------------------------------------------------------------
TEST(TestFrenetPoseConvertion, fromRomeaToRosMsg )
{
  romea::PathFrenetPose2D romea_frenet_pose2d;
  romea_frenet_pose2d.curvilinearAbscissa=1;
  romea_frenet_pose2d.lateralDeviation=2;
  romea_frenet_pose2d.courseDeviation=3;
  for(size_t n=0;n<9;++n)
  {
    romea_frenet_pose2d.covariance(n)=n;
  }

  romea_path_msgs::PathFrenetPose2D ros_frenet_pose2d_msg;
  romea::to_ros_msg(romea_frenet_pose2d,ros_frenet_pose2d_msg);

  EXPECT_DOUBLE_EQ(romea_frenet_pose2d.curvilinearAbscissa,ros_frenet_pose2d_msg.curvilinear_abscissa);
  EXPECT_DOUBLE_EQ(romea_frenet_pose2d.lateralDeviation,ros_frenet_pose2d_msg.lateral_deviation);
  EXPECT_DOUBLE_EQ(romea_frenet_pose2d.courseDeviation,ros_frenet_pose2d_msg.course_deviation);
  for(size_t n=0;n<9;++n)
  {
    EXPECT_DOUBLE_EQ(romea_frenet_pose2d.covariance(n),ros_frenet_pose2d_msg.covariance[n]);
  }
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
