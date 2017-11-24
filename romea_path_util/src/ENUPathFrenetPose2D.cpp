//romea
#include "ENUPathFrenetPose2D.hpp"

//std
#include <iostream>

namespace romea {

//-----------------------------------------------------------------------------
ENUPathFrenetPose2D::ENUPathFrenetPose2D(const double &curvilinearAbscissa,
                           const double &lateralDeviation,
                           const double &courseDeviation,
                           const Eigen::Matrix3d &covariance) :
  curvilinearAbscissa_(curvilinearAbscissa),
  lateralDeviation_(lateralDeviation),
  courseDeviation_(courseDeviation),
  covariance_(covariance)
{

}

//-----------------------------------------------------------------------------
double ENUPathFrenetPose2D::getCurvilinearAbscissa() const
{
  return curvilinearAbscissa_;
}

//-----------------------------------------------------------------------------
double ENUPathFrenetPose2D::getLateralDeviation() const
{
  return lateralDeviation_;
}

//-----------------------------------------------------------------------------
double ENUPathFrenetPose2D::getCourseDeviation() const
{
  return courseDeviation_;
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d ENUPathFrenetPose2D::getCovariance() const
{
  return covariance_;
}

//-----------------------------------------------------------------------------
void ENUPathFrenetPose2D::operator+=(const double & curvilinearAbscissaOffset)
{
  curvilinearAbscissa_+=curvilinearAbscissaOffset;
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const ENUPathFrenetPose2D & frenetPose)
{
  os << "Frenet pose "<< std::endl;
  os << " curvilinear abscissa = " << frenetPose.getCurvilinearAbscissa() <<std::endl;
  os << " lateral deviation = " << frenetPose.getLateralDeviation() <<std::endl;
  os << " course deviation = " << frenetPose.getCourseDeviation() <<std::endl;
  os << " Covariance " <<std::endl;
  os << frenetPose.getCovariance() <<std::endl;
  return os;
}

}
