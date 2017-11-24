#ifndef romea_ENUPathMatching2D_hpp
#define romea_ENUPathMatching2D_hpp

#include "ENUPath2D.hpp"
#include "ENUPathCurve2D.hpp"
#include "ENUPathMatchedPoint2D.hpp"
#include <ENUPose2D.hpp>

namespace romea {

class ENUPathMatching2D
{

public :

  ENUPathMatching2D();

  ENUPathMatching2D(const double & maximalResearchRadius,
                    const double & interpolationWindowLength);

  void setMaximalResearchRadius(const double & maximalResearchRadius);

  void setInterpolationWindowLength(const double & interpolationWindowLength);

  boost::optional<ENUPathMatchedPoint2D> match(const ENUPath2D & path,
                                               const ENUPose2D & vehiclePose);

  boost::optional<ENUPathMatchedPoint2D> match(const ENUPath2D & path,
                                               const ENUPose2D & vehiclePose,
                                               const ENUPathMatchedPoint2D & previousMatchedPoint,
                                               const double & expectedTravelledDistance);
private :





  size_t findNearestPointIndex_(const ENUPath2D & path,
                                const Eigen::Vector2d & vehiclePosition,
                                const Range<size_t> indexRange)const;

  Range<size_t> findIndexRange_(const ENUPath2D & path,
                                const size_t & pointIndex,
                                const double & researchIntervalLength);

  boost::optional<ENUPathMatchedPoint2D> findMatchedPoint_(const ENUPath2D & path,
                                                           const ENUPose2D &vehiclePose,
                                                           const size_t & nearestPointIndex);

private:

  double maximalResearchRadius_;
  double interpolationWindowLength_;
  ENUPathCurve2D interpolatedPath_;

};

}

#endif
