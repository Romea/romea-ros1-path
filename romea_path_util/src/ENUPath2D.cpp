#include "ENUPath2D.hpp"

namespace romea {


//-----------------------------------------------------------------------------
ENUPath2D::ENUPath2D():
  X_(),
  Y_(),
  curvilinearAbscissa_()
{

}

//-----------------------------------------------------------------------------
ENUPath2D::ENUPath2D(const VectorOfEigenVector<Eigen::Vector2d> & points):
  ENUPath2D::ENUPath2D()
{
  load(points);
}

//-----------------------------------------------------------------------------
void ENUPath2D::load(const VectorOfEigenVector<Eigen::Vector2d> &points)
{
  size_t numberOfPoints = points.size();
  assert(numberOfPoints>=2);

  X_.resize(numberOfPoints,0);
  Y_.resize(numberOfPoints,0);
  curvilinearAbscissa_.resize(numberOfPoints,0);

  X_[0]=points[0].x();
  Y_[0]=points[0].y();

  for(size_t n=1;n<numberOfPoints;++n)
  {
    X_[n]=points[n].x();
    Y_[n]=points[n].y();
    curvilinearAbscissa_[n] = curvilinearAbscissa_[n-1]+(points[n]-points[n-1]).norm();
  }
}

//-----------------------------------------------------------------------------
const ENUPath2D::Vector & ENUPath2D::getX()const
{
  return X_;
}

//-----------------------------------------------------------------------------
const ENUPath2D::Vector & ENUPath2D::getY()const
{
  return Y_;
}

//-----------------------------------------------------------------------------
const ENUPath2D::Vector & ENUPath2D::getCurvilinearAbscissa()const
{
  return curvilinearAbscissa_;
}


}
