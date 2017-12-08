//romea
#include "PathMatching2D.hpp"
#include <math/EulerAngles.hpp>

namespace romea {

//-----------------------------------------------------------------------------
PathMatching2D::PathMatching2D():
  PathMatching2D(0,0)
{

}


//-----------------------------------------------------------------------------
PathMatching2D::PathMatching2D(const double & maximalResearchRadius,
                               const double & interpolationWindowLength):
  maximalResearchRadius_(maximalResearchRadius),
  interpolationWindowLength_(interpolationWindowLength)
{

}

//-----------------------------------------------------------------------------
void PathMatching2D::setMaximalResearchRadius(const double & maximalResearchRadius)
{
  maximalResearchRadius_=maximalResearchRadius;
}


//-----------------------------------------------------------------------------
void PathMatching2D::setInterpolationWindowLength(const double & interpolationWindowLength)
{
  interpolationWindowLength_=interpolationWindowLength;
}


//-----------------------------------------------------------------------------
Range<size_t> PathMatching2D::findIndexRange_(const Path2D & path,
                                              const size_t & pointIndex,
                                              const double & researchIntervalLength)

{
  const auto & curvilinearAbscissas = path.getCurvilinearAbscissa();
  assert(pointIndex<curvilinearAbscissas.size());

  size_t minimalIndex = pointIndex;
  size_t maximalIndex = pointIndex;
  double pointCurvilinearAbscissa = curvilinearAbscissas[pointIndex];

  while(minimalIndex !=0 &&
        pointCurvilinearAbscissa - curvilinearAbscissas[minimalIndex] < researchIntervalLength/2.)
  {
    minimalIndex--;
  }

  while(maximalIndex != curvilinearAbscissas.size()-1 &&
        curvilinearAbscissas[maximalIndex] - pointCurvilinearAbscissa < researchIntervalLength/2.)
  {
    maximalIndex++;
  }

  return Range<size_t>(minimalIndex,maximalIndex);
}


//-----------------------------------------------------------------------------
boost::optional<PathMatchedPoint2D> PathMatching2D::findMatchedPoint_(const Path2D & path,
                                                                      const Pose2D & vehiclePose,
                                                                      const size_t & nearestPointIndex)
{

  Range<size_t> indexRange = findIndexRange_(path,nearestPointIndex,interpolationWindowLength_);
  double nearestCurvilinearAbscissa = path.getCurvilinearAbscissa()[nearestPointIndex];

  boost::optional<PathMatchedPoint2D> matchedPoint;
  Eigen::Map<const Eigen::ArrayXd> X(path.getX().data()+indexRange.getMin(),indexRange.interval()+1);
  Eigen::Map<const Eigen::ArrayXd> Y(path.getY().data()+indexRange.getMin(),indexRange.interval()+1);
  Eigen::Map<const Eigen::ArrayXd> S(path.getCurvilinearAbscissa().data()+indexRange.getMin(),indexRange.interval()+1);

  if(interpolatedPath_.estimate(X,Y,S) &&
     interpolatedPath_.findNearestCurvilinearAbscissa(vehiclePose.getPosition(),nearestCurvilinearAbscissa))
  {
    double xp = interpolatedPath_.computeX(nearestCurvilinearAbscissa);
    double yp = interpolatedPath_.computeY(nearestCurvilinearAbscissa);
    double tangent = interpolatedPath_.computeTangent(nearestCurvilinearAbscissa);
    double curvature = interpolatedPath_.computeCurvature(nearestCurvilinearAbscissa);

    const double & xv = vehiclePose.getPosition().x();
    const double & yv = vehiclePose.getPosition().y();
    const double & o = vehiclePose.getCourse();
    const double & cost= std::cos(tangent);
    const double & sint = std::sin(tangent);

    double courseDeviation = betweenMinusPiAndPi(o-tangent);
    double lateralDeviation = (yv-yp)*cost-(xv-xp)*sint;

    Eigen::Matrix3d J=Eigen::Matrix3d::Identity();
    J.block<2,2>(0,0) = eulerAngleToRotation2D(courseDeviation);
    Eigen::Matrix3d frenetPoseCovariance = J*vehiclePose.getCovariance()*J.transpose();

    size_t nearestPointIndex = findNearestPointIndex_(path,Eigen::Vector2d(xp,yp),indexRange);

    //Singularity
    if((std::abs(curvature) > 10e-6)&&(std::abs(lateralDeviation-(1/curvature))<=10e-6))
    {
      curvature = 0;
    }

    matchedPoint = PathMatchedPoint2D(xp,
                                      yp,
                                      tangent,
                                      curvature,
                                      0,
                                      nearestCurvilinearAbscissa,
                                      lateralDeviation,
                                      courseDeviation,
                                      frenetPoseCovariance,
                                      nearestPointIndex);
  }

  return matchedPoint;
}

//-----------------------------------------------------------------------------
size_t PathMatching2D::findNearestPointIndex_(const Path2D & path,
                                              const Eigen::Vector2d & vehiclePosition,
                                              const Range<size_t> indexRange)const
{

  //find neareast point on path
  const auto X = path.getX();
  const auto Y = path.getY();

  size_t nearestPointIndex= indexRange.getMax();
  double minimalDistance = maximalResearchRadius_;

  for(size_t n=indexRange.getMin();n<indexRange.getMax();n++)
  {
    double distance = (vehiclePosition-Eigen::Vector2d(X[n],Y[n])).norm();
    if(distance < minimalDistance){
      minimalDistance=distance;
      nearestPointIndex=n;
    }
  }

  return nearestPointIndex;
}

//-----------------------------------------------------------------------------
boost::optional<PathMatchedPoint2D> PathMatching2D::match(const Path2D & path,
                                                          const Pose2D & vehiclePose)
{

  boost::optional<PathMatchedPoint2D> matchedPoint;

  //find neareast point on path
  size_t numberOfPoints = path.getX().size();
  size_t nearestPointIndex = findNearestPointIndex_(path,vehiclePose.getPosition(),Range<size_t>(0,numberOfPoints));

  //compute matched point
  if(nearestPointIndex!=numberOfPoints)
  {
    matchedPoint = findMatchedPoint_(path,
                                     vehiclePose,
                                     nearestPointIndex);

  }

  return matchedPoint;
}

//-----------------------------------------------------------------------------
boost::optional<PathMatchedPoint2D> PathMatching2D::match(const Path2D & path,
                                                          const Pose2D & vehiclePose,
                                                          const PathMatchedPoint2D & previousMatchedPoint,
                                                          const double & expectedTravelledDistance)
{


  Range<size_t> rangeIndex = findIndexRange_(path,
                                             previousMatchedPoint.getNearestPointIndex(),
                                             expectedTravelledDistance);

  size_t nearestPointIndex = findNearestPointIndex_(path,
                                                    vehiclePose.getPosition(),
                                                    rangeIndex);


  return findMatchedPoint_(path,
                           vehiclePose,
                           nearestPointIndex);

}

}
