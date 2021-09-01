#include "path_matching_base.hpp"


namespace {
const double DEFAULT_MAXIMAL_REASEARCH_RADIUS =10;
const double DEFAULT_INTERPOLATION_WINDOW_LENGTH =3;
const double DEFAULT_PREDICTION_TIME_HORIZON=0.5;
}

namespace romea {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatchingBase::PathMatchingBase():
  prediction_time_horizon_(0),
  maximal_research_radius_(0),
  interpolation_window_length_(0),
  odom_sub_(),
  match_pub_()
{
}

//-----------------------------------------------------------------------------
void PathMatchingBase::configureMatchingInfoPublisher_(ros::NodeHandle & nh)
{
  match_pub_ = nh.advertise<romea_path_msgs::PathMatchingInfo2D>("path_matching_info",1);
}

//-----------------------------------------------------------------------------
void PathMatchingBase::configureOdomSubscriber_(ros::NodeHandle & nh)
{
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("filtered_odom", 1, &PathMatchingBase::processOdom_,this);
}

//-----------------------------------------------------------------------------
void PathMatchingBase::configureDiagnosticPublisher_(ros::NodeHandle & nh)
{
  diagnostic_pub_.init(nh,"path_matching",1.0);
}

//-----------------------------------------------------------------------------
void PathMatchingBase::configureMaximalResearshRadius_(ros::NodeHandle & private_nh)
{
  private_nh.param("maximal_researh_radius",
                   maximal_research_radius_,
                   DEFAULT_MAXIMAL_REASEARCH_RADIUS);
}

//-----------------------------------------------------------------------------
void PathMatchingBase::configureInterpolationWindowLength_(ros::NodeHandle & private_nh)
{
  private_nh.param("interpolation_window_length",
                   interpolation_window_length_,
                   DEFAULT_INTERPOLATION_WINDOW_LENGTH);

//  path_.setInterpolationWindowLength(interpolation_window_length);

}

//-----------------------------------------------------------------------------
void PathMatchingBase::configurePredictionTimeHorizon_(ros::NodeHandle & private_nh)
{
  private_nh.param("prediction_time_horizon",
                   prediction_time_horizon_,
                   DEFAULT_PREDICTION_TIME_HORIZON);

}


}
