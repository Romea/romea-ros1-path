//romea
#include "path_matching_display.hpp"
#include <romea_localisation_utils/RvizDisplay.hpp>

namespace romea {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatchingDisplay::PathMatchingDisplay():
  enable_(true),
  rviz_util_("path","matching"),
  path3d_(),
  interpolatedPath3d_(30)
{
}

//-----------------------------------------------------------------------------
void PathMatchingDisplay::configure(const Path2D &path)
{
  rviz_util_.loadMarkerPub();
  rviz_util_.deleteAllMarkers();
  rviz_util_.enableBatchPublishing();

  size_t number_of_points= path.getX().size();
  path3d_.resize(number_of_points,Eigen::Vector3d::Zero());

  const auto & x = path.getX();
  const auto & y = path.getY();
  for(size_t n =0; n<number_of_points;++n)
  {
    path3d_[n].x()=x[n];
    path3d_[n].y()=y[n];

  }
}


//-----------------------------------------------------------------------------
bool PathMatchingDisplay::enable()
{
  enable_=true;
}

//-----------------------------------------------------------------------------
bool PathMatchingDisplay::disable()
{
  enable_=false;
}

//-----------------------------------------------------------------------------
void PathMatchingDisplay::displayInterpolatedPath_(const PathCurve2D & pathCurve)
{
  double ss=pathCurve.getMinimalCurvilinearAbscissa();
  double ds=(pathCurve.getMaximalCurvilinearAbscissa()-ss)/30.;

  for(size_t n=0; n<30; n++)
  {
    double s = ss+ n*ds;
    interpolatedPath3d_[n].x()=pathCurve.computeX(s);
    interpolatedPath3d_[n].y()=pathCurve.computeY(s);
    interpolatedPath3d_[n].z()=0.1;

  }

  rviz_util_.publishSpheres(interpolatedPath3d_,rviz_visual_tools::RED,rviz_visual_tools::XXLARGE);
}

//-----------------------------------------------------------------------------
void PathMatchingDisplay::display(const Pose2D & vehicle_pose,
                                  const PathCurve2D * pathCurve)
{
  if(enable_)
  {
    rviz_util_.deleteAllMarkers();
    rviz_util_.publishSpheres(path3d_,rviz_visual_tools::WHITE,rviz_visual_tools::XXLARGE);
    romea::publish(rviz_util_,vehicle_pose,rviz_visual_tools::GREEN);

    if(pathCurve!=nullptr)
    {
      displayInterpolatedPath_(*pathCurve);
    }

    rviz_util_.trigger();
  }

}


}
