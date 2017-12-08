
//romea
#include "PathFrenetPose2D.hpp"

//std
#include <iostream>

namespace romea {

//-----------------------------------------------------------------------------
PathFrenetPose2D::PathFrenetPose2D(const double &curvilinearAbscissa,
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
double PathFrenetPose2D::getCurvilinearAbscissa() const
{
  return curvilinearAbscissa_;
}

//-----------------------------------------------------------------------------
double PathFrenetPose2D::getLateralDeviation() const
{
  return lateralDeviation_;
}

//-----------------------------------------------------------------------------
double PathFrenetPose2D::getCourseDeviation() const
{
  return courseDeviation_;
}

//-----------------------------------------------------------------------------
Eigen::Matrix3d PathFrenetPose2D::getCovariance() const
{
  return covariance_;
}

//-----------------------------------------------------------------------------
void PathFrenetPose2D::operator+=(const double & curvilinearAbscissaOffset)
{
  curvilinearAbscissa_+=curvilinearAbscissaOffset;
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const PathFrenetPose2D & frenetPose)
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
