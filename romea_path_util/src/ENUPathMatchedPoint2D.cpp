//romea
#include "ENUPathMatchedPoint2D.hpp"

namespace romea {

//-----------------------------------------------------------------------------
ENUPathMatchedPoint2D::ENUPathMatchedPoint2D(const ENUPathPosture2D &enuPathPosture,
                                             const ENUPathFrenetPose2D & frenetPose,
                                             const double &neareastPointIndex):
  enuPosture_(enuPathPosture),
  frenetPose_(frenetPose),
  neareastPointIndex_(neareastPointIndex)
{

}

//-----------------------------------------------------------------------------
ENUPathMatchedPoint2D::ENUPathMatchedPoint2D(const double & positionAlongXNorthAxis,
                                             const double & positionAlongYEastAxis,
                                             const double & orientationAroundZDownAxis,
                                             const double & curvatureAlongPath,
                                             const double & dotCurvatureAlongPath,
                                             const double & curvilinearAbscissa,
                                             const double & lateralDeviation,
                                             const double & courseDeviation,
                                             const Eigen::Matrix3d & frenetPoseCovariance,
                                             const double &neareastPointIndex):
  enuPosture_(positionAlongXNorthAxis,
              positionAlongYEastAxis,
              orientationAroundZDownAxis,
              curvatureAlongPath,
              dotCurvatureAlongPath),
  frenetPose_(curvilinearAbscissa,
              lateralDeviation,
              courseDeviation,
              frenetPoseCovariance),
  neareastPointIndex_(neareastPointIndex)
{

}

//-----------------------------------------------------------------------------
const ENUPathPosture2D &ENUPathMatchedPoint2D::getENUPosture() const
{
  return enuPosture_;
}

//-----------------------------------------------------------------------------
const ENUPathFrenetPose2D & ENUPathMatchedPoint2D::getFrenetPose() const
{
  return frenetPose_;
}

//-----------------------------------------------------------------------------
const size_t & ENUPathMatchedPoint2D::getNearestPointIndex() const
{
  return neareastPointIndex_;
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const ENUPathMatchedPoint2D & matchedPoint)
{
  os << "Matched point "<< std::endl;
  os << matchedPoint.getENUPosture();
  os << matchedPoint.getFrenetPose();
  os << "nearest point index " << matchedPoint.getNearestPointIndex() << std::endl;
  return os;
}


}//End of namespace romea


