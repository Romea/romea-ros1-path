#ifndef romea_ENUPathCurve2D_hpp
#define romea_ENUPathCurve2D_hpp

//romea
#include <PathPosture2D.hpp>
#include <PathFrenetPose2D.hpp>
#include <math/Range.hpp>

//std
#include <vector>

namespace romea {


class PathCurve2D
{

public :

    using Vector = std::vector<double,Eigen::aligned_allocator<double> > ;

public :

  PathCurve2D();

  bool estimate(const Vector & X,
                const Vector & Y,
                const Vector & S ,
                const Range<size_t> & indexRange);

  bool findNearestCurvilinearAbscissa(const Eigen::Vector2d & vehiclePosition,
                                      double & curvilinearAbscissa) const;

  double computeX(const double & curvilinearAbscissa)const;

  double computeY(const double & curvilinearAbscissa)const;

  double computeTangent(const double & curvilinearAbscissa)const;

  double computeCurvature(const double & curvilinearAbscissa)const;

  const double & getMinimalCurvilinearAbscissa()const;

  const double & getMaximalCurvilinearAbscissa()const;

  const Range<size_t> & getIndexRange()const;

private :

  Eigen::Array3d fxPolynomCoefficient_;
  Eigen::Array3d fyPolynomCoefficient_;
  double minimalCurvilinearAbscissa_;
  double maximalCurvilinearAbscissa_;
  Range<size_t> indexRange_;


};

}


#endif
