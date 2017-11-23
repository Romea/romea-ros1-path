//romea
#include "ENUPathMatching2D.hpp"
#include <math/EulerAngles.hpp>

namespace romea {

//-----------------------------------------------------------------------------
void ENUPathMatching2D::setMaximalResearchRadius(const double & maximalResearchRadius)
{
  maximalResearchRadius_=maximalResearchRadius;
}


//-----------------------------------------------------------------------------
void ENUPathMatching2D::setInterpolationWindowLength(const double & interpolationWindowLength)
{
  interpolationWindowLength_=interpolationWindowLength;
}


//-----------------------------------------------------------------------------
Range<size_t> ENUPathMatching2D::computeInterpolationWindowIndexRange_(const ENUPath2D & path,
                                                                       const size_t & pointIndex,
                                                                       const double & expectedTravelledDistance)

{
  const auto & curvilinearAbscissas = path.getCurvilinearAbscissa();
  assert(pointIndex<curvilinearAbscissas.size());

  size_t minimalIndex = pointIndex;
  size_t maximalIndex = pointIndex;
  double pointCurvilinearAbscissa = expectedTravelledDistance+curvilinearAbscissas[pointIndex];

  while(minimalIndex !=0 &&
        pointCurvilinearAbscissa - curvilinearAbscissas[minimalIndex] < interpolationWindowLength_/2.)
  {
    minimalIndex--;
  }

  while(maximalIndex != curvilinearAbscissas.size()-1 &&
        curvilinearAbscissas[minimalIndex] - pointCurvilinearAbscissa < interpolationWindowLength_/2.)
  {
    maximalIndex++;
  }

  return Range<size_t>(minimalIndex,maximalIndex);
}


//-----------------------------------------------------------------------------
boost::optional<ENUPathMatchedPoint2D> ENUPathMatching2D::findMatchedPoint_(const ENUPath2D & path,
                                                                            const ENUPose2D & vehiclePose,
                                                                            const Range<size_t> & indexRange,
                                                                            double nearestCurvilinearAbscissa)
{
  boost::optional<ENUPathMatchedPoint2D> matchedPoint;

  Eigen::Map<const Eigen::ArrayXd> X(path.getX().data()+indexRange.getMin(),indexRange.interval());
  Eigen::Map<const Eigen::ArrayXd> Y(path.getY().data()+indexRange.getMin(),indexRange.interval());
  Eigen::Map<const Eigen::ArrayXd> S(path.getCurvilinearAbscissa().data()+indexRange.getMin(),indexRange.interval());

  if(interpolatedPath_.estimate(X,Y,S) &&
     interpolatedPath_.findNearestCurvilinearAbscissa(vehiclePose.getPosition(),nearestCurvilinearAbscissa))
  {
    double xp = interpolatedPath_.computeX(nearestCurvilinearAbscissa);
    double yp = interpolatedPath_.computeY(nearestCurvilinearAbscissa);
    double tangent = interpolatedPath_.computeTangent(nearestCurvilinearAbscissa);
    double curvature = interpolatedPath_.computeCurvature(nearestCurvilinearAbscissa);

    const double & xv = vehiclePose.getPosition().x();
    const double & yv = vehiclePose.getPosition().y();
    const double & o = vehiclePose.getOrientationAroundZDownAxis();
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

    matchedPoint = ENUPathMatchedPoint2D(xp,
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
size_t ENUPathMatching2D::findNearestPointIndex_(const ENUPath2D & path,
                                                 const Eigen::Vector2d & vehiclePosition,
                                                 const Range<size_t> indexRange)const
{

  //find neareast point on path
  const auto X = path.getX();
  const auto Y = path.getY();

  size_t nearestPointIndex= X.size();
  double minimalDistance = maximalResearchRadius_;

  for(size_t n=indexRange.getMin();n<X.size();indexRange.getMax())
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
boost::optional<ENUPathMatchedPoint2D> ENUPathMatching2D::match(const ENUPath2D & path,
                                                                const ENUPose2D & vehiclePose)
{

  boost::optional<ENUPathMatchedPoint2D> matchedPoint;

  //find neareast point on path
  size_t numberOfPoints = path.getX().size();
  size_t nearestPointIndex = findNearestPointIndex_(path,vehiclePose.getPosition(),Range<size_t>(0,numberOfPoints));

  //compute matched point
  if(nearestPointIndex!=numberOfPoints)
  {

    Range<size_t> rangeIndex = computeInterpolationWindowIndexRange_(path,nearestPointIndex,0);
    double nearestCurvilinearAbscissa = path.getCurvilinearAbscissa()[nearestPointIndex];

    matchedPoint = findMatchedPoint_(path,
                                     vehiclePose,
                                     rangeIndex,
                                     nearestCurvilinearAbscissa);

  }

  return matchedPoint;
}

//-----------------------------------------------------------------------------
boost::optional<ENUPathMatchedPoint2D> ENUPathMatching2D::match(const ENUPath2D & path,
                                                                const ENUPose2D & vehiclePose,
                                                                const ENUPathMatchedPoint2D & previousMatchedPoint,
                                                                const double & expectedTravelledDistance)
{


  const size_t nearestPointIndex = previousMatchedPoint.getNearestPointIndex();

  Range<size_t> rangeIndex = computeInterpolationWindowIndexRange_(path,
                                                                   nearestPointIndex,
                                                                   expectedTravelledDistance);

  double nearestCurvilinearAbscissa = previousMatchedPoint.getFrenetPose().getCurvilinearAbscissa();

  return findMatchedPoint_(path,
                           vehiclePose,
                           rangeIndex,
                           nearestCurvilinearAbscissa);

}

}
