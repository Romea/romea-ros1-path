#ifndef romea_ENUPathFrenetPose2D_hpp
#define romea_ENUPathFrenetPose2D_hpp

//eigen
#include <Eigen/Core>

//boost to replace by std optional
#include <boost/optional/optional.hpp>

//std
#include <memory>

namespace romea {

class ENUPathFrenetPose2D {

public :

  typedef boost::optional<ENUPathFrenetPose2D> Opt;
  typedef std::shared_ptr<ENUPathFrenetPose2D> Ptr;

public :

  ENUPathFrenetPose2D(const double &curvilinearAbscissa,
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

std::ostream& operator<<(std::ostream & os, const ENUPathFrenetPose2D & frenetPose);

}

#endif
