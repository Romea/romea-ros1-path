#ifndef romea_ENUPathMatchedPoint2D_hpp
#define romea_ENUPathMatchedPoint2D_hpp

//romea
#include "PathPosture2D.hpp"
#include "PathFrenetPose2D.hpp"
#include <time/Time.hpp>

//boost to replace by std optional
#include <boost/optional/optional.hpp>

namespace romea {

class PathMatchedPoint2D {

public :

  using Ptr = std::shared_ptr<PathMatchedPoint2D> ;
  using Opt = boost::optional<PathMatchedPoint2D> ;
  using Stamped = StampedWrapper<Duration,PathMatchedPoint2D> ;

public :

  PathMatchedPoint2D(const PathPosture2D & enuPathPosture,
                     const PathFrenetPose2D & frenetPose,
                     const double & neareastPointIndex);

  PathMatchedPoint2D(const double & positionAlongXNorthAxis,
                     const double & positionAlongYEastAxis,
                     const double & orientationAroundZDownAxis,
                     const double & curvature,
                     const double & dotCurvature,
                     const double & curvilinearAbscissa,
                     const double & lateralDeviation,
                     const double & courseDeviation,
                     const Eigen::Matrix3d & frenetPoseCovariance,
                     const double &neareastPointIndex);


public :

  const PathPosture2D & getPosture() const;

  const PathFrenetPose2D & getFrenetPose() const;

  const size_t & getNearestPointIndex() const;

private :

  PathPosture2D enuPosture_;
  PathFrenetPose2D frenetPose_;
  size_t neareastPointIndex_;

};

std::ostream& operator<<(std::ostream & os, const PathMatchedPoint2D & matchedPoint);

}//End of namespace romea

#endif
