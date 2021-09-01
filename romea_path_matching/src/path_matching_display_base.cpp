//romea
#include "path_matching_display_base.hpp"


namespace romea {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatchingDisplayBase::PathMatchingDisplayBase():
  rviz_util_("path","matching"),
  path3d_(),
  interpolatedPath3d_(30),
  is_display_activated_(false)
{
  path3d_.reserve(100000);
  rviz_util_.loadMarkerPub();
  rviz_util_.deleteAllMarkers();
  rviz_util_.enableBatchPublishing();

}

//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::init(ros::NodeHandle private_nh)
{
  private_nh.param("display",is_display_activated_,false);
}

//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::reset()
{
  path3d_.clear();
  interpolatedPath3d_.clear();
}


//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::displayCurveMarkers(const PathCurve2D & path_curve)
{

  double ss=path_curve.getCurvilinearAbscissaInterval().lower();
  double ds=(path_curve.getCurvilinearAbscissaInterval().upper()-ss)/30.;

  for(size_t n=0; n<30; n++)
  {
    double s = ss+ n*ds;
    interpolatedPath3d_[n].x()=path_curve.computeX(s);
    interpolatedPath3d_[n].y()=path_curve.computeY(s);
    interpolatedPath3d_[n].z()=0.1;
  }

  rviz_util_.publishSpheres(interpolatedPath3d_,rviz_visual_tools::RED,rviz_visual_tools::XXLARGE);
}

//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::displayPathMarkers(const Pose2D & vehicle_pose)
{
  rviz_util_.publishSpheres(path3d_,rviz_visual_tools::WHITE,rviz_visual_tools::XXLARGE);
  romea::publish(rviz_util_,vehicle_pose,rviz_visual_tools::GREEN);
}

//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::deleteMarkers()
{
  rviz_util_.deleteAllMarkers();
}

//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::trigger()
{
  if(is_display_activated_)
  {
    rviz_util_.trigger();
  }
}

////-----------------------------------------------------------------------------
//void PathMatchingDisplayBase::display(const Pose2D & vehicle_pose)
//{
//  if(is_display_activated_)
//  {
//    deleteMarkers_();
//    displayPathMarkers_(vehicle_pose);
//    trigger_();
//  }
//}

////-----------------------------------------------------------------------------
//void PathMatchingDisplayBase::display(const Pose2D & vehicle_pose,
//                                      const PathCurve2D & path_curve)
//{

//    if(is_display_activated_)
//    {
//      deleteMarkers_();
//      displayPathMarkers_(vehicle_pose);
//      displayCurveMarkers_(path_curve);
//      trigger_();
//    }
//}



}
