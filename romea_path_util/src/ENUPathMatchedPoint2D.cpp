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


}//End of namespace romea


