#ifndef romea_ENUPathMatchedPoint2D_hpp
#define romea_ENUPathMatchedPoint2D_hpp

//romea
#include "ENUPathPosture2D.hpp"
#include "ENUPathFrenetPose2D.hpp"
#include <time/Time.hpp>

//boost to replace by std optional
#include <boost/optional/optional.hpp>

namespace romea {

class ENUPathMatchedPoint2D {

public :

  typedef std::shared_ptr<ENUPathMatchedPoint2D> Ptr;
  typedef boost::optional<ENUPathMatchedPoint2D> Opt;
  typedef StampedWrapper<Duration,ENUPathMatchedPoint2D> Stamped;

public :

  ENUPathMatchedPoint2D(const ENUPathPosture2D & enuPathPosture,
                        const ENUPathFrenetPose2D & frenetPose,
                        const double & neareastPointIndex);

  ENUPathMatchedPoint2D(const double & positionAlongXNorthAxis,
                        const double & positionAlongYEastAxis,
                        const double & orientationAroundZDownAxis,
                        const double & curvatureAlongPath,
                        const double & dotCurvatureAlongPath,
                        const double & curvilinearAbscissa,
                        const double & lateralDeviation,
                        const double & courseDeviation,
                        const Eigen::Matrix3d & frenetPoseCovariance,
                        const double &neareastPointIndex);


public :

  const ENUPathPosture2D & getENUPosture() const;

  const ENUPathFrenetPose2D & getFrenetPose() const;

  const size_t & getNearestPointIndex() const;

private :

  ENUPathPosture2D enuPosture_;
  ENUPathFrenetPose2D frenetPose_;
  size_t neareastPointIndex_;

};

std::ostream& operator<<(std::ostream & os, const ENUPathMatchedPoint2D & matchedPoint);

}//End of namespace romea

#endif

