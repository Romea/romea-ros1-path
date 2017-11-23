#ifndef romea_ENUPath2D_hpp
#define romea_ENUPath2D_hpp

//romea
#include <containers/Eigen/VectorOfEigenVector.hpp>
#include <containers/Range.hpp>

//boost
#include <boost/optional.hpp>

namespace romea {

class ENUPath2D
{
public:

  typedef std::vector<double,Eigen::aligned_allocator<double> > Vector;

public:

  ENUPath2D();

  ENUPath2D(const VectorOfEigenVector<Eigen::Vector2d> & points);

  void load(const VectorOfEigenVector<Eigen::Vector2d> & points);

  const Vector & getX()const;
  const Vector & getY()const;
  const Vector & getCurvilinearAbscissa()const;

private :

  Vector X_;
  Vector Y_;
  Vector curvilinearAbscissa_;
};

}

#endif
