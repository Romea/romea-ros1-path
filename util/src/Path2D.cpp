#include "Path2D.hpp"

namespace romea {


//-----------------------------------------------------------------------------
Path2D::Path2D():
  X_(),
  Y_(),
  curvilinearAbscissa_()
{

}

//-----------------------------------------------------------------------------
Path2D::Path2D(const VectorOfEigenVector<Eigen::Vector2d> & points):
  Path2D::Path2D()
{
  load(points);
}

//-----------------------------------------------------------------------------
void Path2D::load(const VectorOfEigenVector<Eigen::Vector2d> &points)
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
const Path2D::Vector & Path2D::getX()const
{
  return X_;
}

//-----------------------------------------------------------------------------
const Path2D::Vector & Path2D::getY()const
{
  return Y_;
}

//-----------------------------------------------------------------------------
const Path2D::Vector & Path2D::getCurvilinearAbscissa()const
{
  return curvilinearAbscissa_;
}


}
