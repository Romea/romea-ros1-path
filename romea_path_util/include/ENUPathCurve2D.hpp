#ifndef romea_ENUPathCurve2D_hpp
#define romea_ENUPathCurve2D_hpp

//romea
#include <ENUPathPosture2D.hpp>
#include <ENUPathFrenetPose2D.hpp>

namespace romea {


class ENUPathCurve2D
{

public :

  ENUPathCurve2D();

  bool estimate(const Eigen::ArrayXd & x,const Eigen::ArrayXd & y, const Eigen::ArrayXd & cuvilinerAbscissas);

  bool findNearestCurvilinearAbscissa(const Eigen::Vector2d & vehiclePosition,double & curvilinearAbscissa) const;

  double computeX(const double & curvilinearAbscissa)const;

  double computeY(const double & curvilinearAbscissa)const;

  double computeTangent(const double & curvilinearAbscissa)const;

  double computeCurvature(const double & curvilinearAbscissa)const;

private :

  Eigen::Array3d fxPolynomCoefficient_;
  Eigen::Array3d fyPolynomCoefficient_;
  double minimalCurvilinearAbscissa_;
  double maximalCurvilinearAbscissa_;

};

}


#endif
