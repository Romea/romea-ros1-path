#include "PathPosture2D.hpp"
#include <math/EulerAngles.hpp>

namespace romea {

//-----------------------------------------------------------------------------
PathPosture2D::PathPosture2D():
  PathPosture2D(Eigen::Vector2d::Zero(),0.,0.,0.)
{
}

//-----------------------------------------------------------------------------
PathPosture2D::PathPosture2D(const Eigen::Vector2d & position,
                             const double & course,
                             const double & curvature):
  PathPosture2D(position,course,curvature,0.)
{
}
//-----------------------------------------------------------------------------
PathPosture2D::PathPosture2D(const double & x,
                             const double & y,
                             const double & course,
                             const double & curvature):
  PathPosture2D(Eigen::Vector2d(x,y),
                course,curvature,0.)
{
}

//-----------------------------------------------------------------------------
PathPosture2D::PathPosture2D(const Eigen::Vector2d &position,
                             const double &course,
                             const double &curvature,
                             const double &dotCurvature):
  position_(position),
  course_(between0And2Pi(course)),
  curvature_(curvature),
  dotCurvature_(dotCurvature)
{
}

//-----------------------------------------------------------------------------
PathPosture2D::PathPosture2D(const double & x,
                             const double & y,
                             const double & course,
                             const double & curvature,
                             const double & dotCurvature):
  PathPosture2D(Eigen::Vector2d(x,y),
                course,curvature,dotCurvature)
{

}

//-----------------------------------------------------------------------------
double PathPosture2D::getCurvature() const
{
  return curvature_;
}

//-----------------------------------------------------------------------------
double PathPosture2D::getDotCurvature() const
{
  return dotCurvature_;
}

//-----------------------------------------------------------------------------
double PathPosture2D::getCourse() const
{
  return course_;
}

//-----------------------------------------------------------------------------

Eigen::Vector2d PathPosture2D::getPosition() const
{
  return position_;
}

//-----------------------------------------------------------------------------
double PathPosture2D::getX() const
{
  return position_.x();
}

//-----------------------------------------------------------------------------
double PathPosture2D::getY() const
{
  return position_.y();
}

//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream & os, const PathPosture2D & posture)
{
  os<< "Posture "<< std::endl;
  os<< " x = " << posture.getX() <<std::endl;
  os<< " y = " << posture.getY() <<std::endl;
  os<< " course = " << posture.getCourse() <<std::endl;
  os<< " curvature " << posture.getCurvature() <<std::endl;
  os<< " dot curvature " << posture.getDotCurvature() <<std::endl;
  return os;
}


}
