// gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//romea
#include "ENUPathMatching2D.hpp"
//#include <Tools/Path/trajectory2d/trajectory2d.hpp>

//std
#include <fstream>

romea::VectorOfEigenVector<Eigen::Vector2d> loadPath(const std::string &filename)
{
  romea::VectorOfEigenVector<Eigen::Vector2d> points;
  points.reserve(1000);

  std::string path = std::string(TEST_DIR);
  std::ifstream data(path +filename);

  Eigen::Vector2d p;
  while(!data.eof())
  {
    data >> p[0] >> p[1];
    points.push_back(p);
  }

  return points;
}



TEST(TestPath, testMatching)
{

  const double maximalResearchRadius=10;
  const double interpolationWindowLength=3;

  romea::ENUPose2D firstVehiclePose(-8.2,16.1,120/180.*M_PI,Eigen::Matrix3d::Zero());
  romea::ENUPose2D secondVehiclePose(-9.3,17.2,130/180.*M_PI,Eigen::Matrix3d::Zero());

  romea::ENUPathMatching2D pathmatching(maximalResearchRadius,interpolationWindowLength);

  //load path
  romea::VectorOfEigenVector<Eigen::Vector2d> points= loadPath("/path.txt");
  romea::ENUPath2D rPath(points);


  //match romea
  boost::optional<romea::ENUPathMatchedPoint2D>  rFirstMatchedPoint = pathmatching.match(rPath,firstVehiclePose);
  ASSERT_EQ(rFirstMatchedPoint.is_initialized(),true);

  EXPECT_NEAR(rFirstMatchedPoint->getENUPosture().getPositionAlongXEastAxis(),-8.10917,0.001);
  EXPECT_NEAR(rFirstMatchedPoint->getENUPosture().getPositionAlongYNorthAxis(),16.2537,0.001);
  EXPECT_NEAR(rFirstMatchedPoint->getENUPosture().getOrientationAroundZUpAxis(),2.60782,0.001);
  EXPECT_NEAR(rFirstMatchedPoint->getENUPosture().getCurvatureAlongPath(),0.0816672,0.001);

  EXPECT_NEAR(rFirstMatchedPoint->getFrenetPose().getCurvilinearAbscissa(),17.1109,0.001);
  EXPECT_NEAR(rFirstMatchedPoint->getFrenetPose().getLateralDeviation(),0.17852,0.001);
  EXPECT_NEAR(rFirstMatchedPoint->getFrenetPose().getCourseDeviation(),-0.51343,0.001);

  EXPECT_EQ(rFirstMatchedPoint->getNearestPointIndex(),177);


  boost::optional<romea::ENUPathMatchedPoint2D>  rSecondMatchedPoint = pathmatching.match(rPath,secondVehiclePose,*rFirstMatchedPoint,10.)  ;
  ASSERT_EQ(rSecondMatchedPoint.is_initialized(),true);

  EXPECT_NEAR(rSecondMatchedPoint->getENUPosture().getPositionAlongXEastAxis(),-9.41664,0.001);
  EXPECT_NEAR(rSecondMatchedPoint->getENUPosture().getPositionAlongYNorthAxis(),16.9346,0.001);
  EXPECT_NEAR(rSecondMatchedPoint->getENUPosture().getOrientationAroundZUpAxis(),2.72756,0.001);
  EXPECT_NEAR(rSecondMatchedPoint->getENUPosture().getCurvatureAlongPath(),0.104456,0.001);

  EXPECT_NEAR(rSecondMatchedPoint->getFrenetPose().getCurvilinearAbscissa(), 18.5847,0.001);
  EXPECT_NEAR(rSecondMatchedPoint->getFrenetPose().getLateralDeviation(),-0.289936,0.001);
  EXPECT_NEAR(rSecondMatchedPoint->getFrenetPose().getCourseDeviation(),-0.458636,0.001);


//  irsteaai load path
//  irsteaai1::Trajectory2D iPath;
//  for(const auto & point :points)
//  {
//    iPath.addPoint(point[0],point[1]);
//  }

//  //match irsteaai
//  double iLateralDeviation;
//  double iCourseDeviation;
//  double iCurvature;

//  std::cout << "\n irsteaai first match " << std::endl;
//  iPath.setActiveWindow(interpolationWindowLength);
//  bool iMatch=iPath.computeErrorToTraj(firstVehiclePose.getPositionAlongXEastAxis(),
//                                       firstVehiclePose.getPositionAlongYNorthAxis(),
//                                       firstVehiclePose.getOrientationAroundZDownAxis(),
//                                       iLateralDeviation,
//                                       iCourseDeviation,
//                                       iCurvature);

//  ASSERT_EQ(iMatch,true);

//  std::cout << " iLateralDeviation " << iLateralDeviation<<std::endl;
//  std::cout << " iCourseDeviation " << iCourseDeviation<<std::endl;
//  std::cout << " iCurvature " << iCurvature<<std::endl;

//  std::cout << "\n irsteaai second match " << std::endl;

//  iMatch=iPath.computeErrorToTraj(secondVehiclePose.getPositionAlongXEastAxis(),
//                                 secondVehiclePose.getPositionAlongYNorthAxis(),
//                                 secondVehiclePose.getOrientationAroundZDownAxis(),
//                                 iLateralDeviation,
//                                 iCourseDeviation,
//                                 iCurvature);

//  ASSERT_EQ(iMatch,true);


//  std::cout << " iLateralDeviation " << iLateralDeviation<<std::endl;
//  std::cout << " iCourseDeviation " << iCourseDeviation<<std::endl;
//  std::cout << " iCurvature " << iCurvature<<std::endl;


}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
