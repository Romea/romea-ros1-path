//romea
#include "PathMatchedPoint2D.hpp"

namespace romea {

//-----------------------------------------------------------------------------
PathMatchedPoint2D::PathMatchedPoint2D(const PathPosture2D &enuPathPosture,
                                       const PathFrenetPose2D & frenetPose,
                                       const double &neareastPointIndex):
  enuPosture_(enuPathPosture),
  frenetPose_(frenetPose),
  neareastPointIndex_(neareastPointIndex)
{

}

//-----------------------------------------------------------------------------
PathMatchedPoint2D::PathMatchedPoint2D(const double & positionAlongXNorthAxis,
                                       const double & positionAlongYEastAxis,
                                       const double & orientationAroundZDownAxis,
                                       const double & curvature,
                                       const double & dotCurvature,
                                       const double & curvilinearAbscissa,
                                       const double & lateralDeviation,
                                       const double & courseDeviation,
                                       const Eigen::Matrix3d & frenetPoseCovariance,
                                       const double &neareastPointIndex):
  enuPosture_(positionAlongXNorthAxis,
              positionAlongYEastAxis,
              orientationAroundZDownAxis,
              curvature,
              dotCurvature),
  frenetPose_(curvilinearAbscissa,
              lateralDeviation,
              courseDeviation,
              frenetPoseCovariance),
  neareastPointIndex_(neareastPointIndex)
{

}

//-----------------------------------------------------------------------------
const PathPosture2D &PathMatchedPoint2D::getPosture() const
{
  return enuPosture_;
}

//-----------------------------------------------------------------------------
const PathFrenetPose2D & PathMatchedPoint2D::getFrenetPose() const
{
  return frenetPose_;
}

//-----------------------------------------------------------------------------
const size_t & PathMatchedPoint2D::getNearestPointIndex() const
{
  return neareastPointIndex_;
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const PathMatchedPoint2D & matchedPoint)
{
  os << "Matched point "<< std::endl;
  os << matchedPoint.getPosture();
  os << matchedPoint.getFrenetPose();
  os << "nearest point index " << matchedPoint.getNearestPointIndex() << std::endl;
  return os;
}


}//End of namespace romea


