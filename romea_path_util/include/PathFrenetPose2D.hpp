#ifndef romea_ENUPathFrenetPose2D_hpp
#define romea_ENUPathFrenetPose2D_hpp

//eigen
#include <Eigen/Core>

//boost to replace by std optional
#include <boost/optional/optional.hpp>

//std
#include <memory>

namespace romea {

class PathFrenetPose2D {

public :

  typedef boost::optional<PathFrenetPose2D> Opt;
  typedef std::shared_ptr<PathFrenetPose2D> Ptr;

public :

  PathFrenetPose2D(const double &curvilinearAbscissa,
                      const double &lateralDeviation,
                      const double &courseDeviation,
                      const Eigen::Matrix3d &covariance);

  double getCurvilinearAbscissa() const;
  double getLateralDeviation() const;
  double getCourseDeviation() const;

  Eigen::Matrix3d getCovariance() const;

  void operator+=(const double & curvilinearAbscissaOffset);

private :

  double curvilinearAbscissa_;
  double lateralDeviation_;
  double courseDeviation_;
  Eigen::Matrix3d covariance_;
};

std::ostream& operator<<(std::ostream & os, const PathFrenetPose2D & frenetPose);

}

#endif
