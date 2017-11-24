#include "ENUPathPosture2D.hpp"
#include <math/EulerAngles.hpp>

namespace romea {

//-----------------------------------------------------------------------------
ENUPathPosture2D::ENUPathPosture2D():
  ENUPathPosture2D(Eigen::Vector2d::Zero(),0.,0.,0.)
{
}

//-----------------------------------------------------------------------------
ENUPathPosture2D::ENUPathPosture2D(const Eigen::Vector2d & position,
                                   const double & orientationAroundZAxis,
                                   const double & curvatureAlongPath):
  ENUPathPosture2D(position,orientationAroundZAxis,curvatureAlongPath,0.)
{
}
//-----------------------------------------------------------------------------
ENUPathPosture2D::ENUPathPosture2D(const double & positionAlongXAxis,
                                   const double & positionAlongYAxis,
                                   const double & orientationAroundZAxis,
                                   const double & curvatureAlongPath):
  ENUPathPosture2D(Eigen::Vector2d(positionAlongXAxis,positionAlongYAxis),
                   orientationAroundZAxis,curvatureAlongPath,0.)
{
}

//-----------------------------------------------------------------------------
ENUPathPosture2D::ENUPathPosture2D(const Eigen::Vector2d &position,
                                   const double &orientationAroundZAxis,
                                   const double &curvatureAlongPath,
                                   const double &dotCurvatureAlongPath):
  position_(position),
  orientationAroundZAxis_(between0And2Pi(orientationAroundZAxis)),
  curvatureAlongPath_(curvatureAlongPath),
  dotCurvatureAlongPath_(dotCurvatureAlongPath)
{
}

//-----------------------------------------------------------------------------
ENUPathPosture2D::ENUPathPosture2D(const double & positionAlongXAxis,
                                   const double & positionAlongYAxis,
                                   const double & orientationAroundZAxis,
                                   const double & curvatureAlongPath,
                                   const double & dotCurvatureAlongPath):
  ENUPathPosture2D(Eigen::Vector2d(positionAlongXAxis,positionAlongYAxis),
                   orientationAroundZAxis,curvatureAlongPath,dotCurvatureAlongPath)
{

}

//-----------------------------------------------------------------------------
double ENUPathPosture2D::getCurvatureAlongPath() const
{
  return curvatureAlongPath_;
}

//-----------------------------------------------------------------------------
double ENUPathPosture2D::getDotCurvatureAlongPath() const
{
  return dotCurvatureAlongPath_;
}

//-----------------------------------------------------------------------------
double ENUPathPosture2D::getOrientationAroundZUpAxis() const
{
  return orientationAroundZAxis_;
}

//-----------------------------------------------------------------------------

Eigen::Vector2d ENUPathPosture2D::getPosition() const
{
  return position_;
}

//-----------------------------------------------------------------------------
double ENUPathPosture2D::getPositionAlongXEastAxis() const
{
  return position_.x();
}

//-----------------------------------------------------------------------------
double ENUPathPosture2D::getPositionAlongYNorthAxis() const
{
  return position_.y();
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const ENUPathPosture2D & posture)
{
  os<< "Posture "<< std::endl;
  os<< " x = " << posture.getPositionAlongXEastAxis() <<std::endl;
  os<< " y = " << posture.getPositionAlongYNorthAxis() <<std::endl;
  os<< " theta = " << posture.getOrientationAroundZUpAxis() <<std::endl;
  os<< " curvature " << posture.getCurvatureAlongPath() <<std::endl;
  os<< " dot curvature " << posture.getDotCurvatureAlongPath() <<std::endl;
  return os;
}


}
