#ifndef romea_ENUPathMatching2D_hpp
#define romea_ENUPathMatching2D_hpp

#include "Path2D.hpp"
#include "PathCurve2D.hpp"
#include "PathMatchedPoint2D.hpp"
#include <Pose2D.hpp>

namespace romea {

class PathMatching2D
{

public :

  PathMatching2D();

  PathMatching2D(const double & maximalResearchRadius,
                 const double & interpolationWindowLength);

  void setMaximalResearchRadius(const double & maximalResearchRadius);

  void setInterpolationWindowLength(const double & interpolationWindowLength);

  boost::optional<PathMatchedPoint2D> match(const Path2D & path,
                                            const Pose2D & vehiclePose);

  boost::optional<PathMatchedPoint2D> match(const Path2D & path,
                                            const Pose2D & vehiclePose,
                                            const PathMatchedPoint2D & previousMatchedPoint,
                                            const double & expectedTravelledDistance);

  const PathCurve2D & getInterpolatedPath() const;

private :


  size_t findNearestPointIndex_(const Path2D & path,
                                const Eigen::Vector2d & vehiclePosition,
                                const Range<size_t> indexRange)const;

  Range<size_t> findIndexRange_(const Path2D & path,
                                const size_t & pointIndex,
                                const double & researchIntervalLength);

  boost::optional<PathMatchedPoint2D> findMatchedPoint_(const Path2D & path,
                                                        const Pose2D &vehiclePose,
                                                        const size_t & nearestPointIndex);

private:

  double maximalResearchRadius_;
  double interpolationWindowLength_;
  PathCurve2D interpolatedPath_;

};

}

#endif
