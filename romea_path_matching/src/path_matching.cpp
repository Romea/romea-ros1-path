//romea
#include "path_matching.hpp"
#include "path_file.hpp"

namespace romea {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatching::PathMatching():
  tf_(),
  display_(),
  diagnostics_(),
  path_(nullptr),
  matched_points_(),
  tracked_matched_point_index_(0)
{
}

//-----------------------------------------------------------------------------
void PathMatching::init(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
  configureMaximalResearshRadius_(private_nh);
  configureInterpolationWindowLength_(private_nh);
  configurePredictionTimeHorizon_(private_nh);
  configureMatchingInfoPublisher_(nh);
  configureDiagnosticPublisher_(nh);
  configureOdomSubscriber_(nh);
  display_.init(private_nh);
  tf_.init(private_nh);
}


//-----------------------------------------------------------------------------
void PathMatching::loadPath(const std::string & filename)
{
  diagnostics_.setPathFilename(filename);

  PathFile path_file(filename);
  path_ = std::make_unique<Path2D>(path_file.getWayPoints(),interpolation_window_length_);
  tf_.setWorldToPathTransformation(path_file.getWorldToPathTransformation());
  display_.loadWayPoints(path_file.getWayPoints());

  diagnostics_.updatePathStatus(true);

}

//-----------------------------------------------------------------------------
void PathMatching::reset()
{
  display_.reset();
  matched_points_.clear();
  tf_.reset();
}

//-----------------------------------------------------------------------------
bool PathMatching::tryToEvaluteMapToPathTransformation_(const ros::Time & stamp,
                                                        const std::string & map_frame_id)
{
  bool status=tf_.evaluteMapToPathTransformation(stamp,map_frame_id);
  diagnostics_.updateLookupTransformStatus(status);
  return status;
}

//-----------------------------------------------------------------------------
void PathMatching::processOdom_(const nav_msgs::Odometry::ConstPtr &msg)
{

  //  std::cout << msg->header.stamp <<"process odom"<<std::endl;

  PoseAndTwist3D enuPoseAndBodyTwist3D;
  toRomea(msg->pose,enuPoseAndBodyTwist3D.pose);
  toRomea(msg->twist,enuPoseAndBodyTwist3D.twist);
  diagnostics_.updateOdomRate(romea::toRomeaDuration(msg->header.stamp));


  if(tryToEvaluteMapToPathTransformation_(msg->header.stamp,msg->header.frame_id))
  {
    const auto & map_to_path = tf_.getMapToPathTransformation();
    romea::Pose2D vehicle_pose = toPose2D(map_to_path*enuPoseAndBodyTwist3D.pose);
    romea::Twist2D vehicle_twist = toTwist2D(enuPoseAndBodyTwist3D.twist);

    if(tryToMatchOnPath_(vehicle_pose,vehicle_twist))
    {
      match_pub_.publish(romea::toRosMsg(msg->header.stamp,
                                         matched_points_,
                                         tracked_matched_point_index_,
                                         path_->getLength(),
                                         vehicle_twist));
    }

    displayResults_(vehicle_pose);
  }
}

//-----------------------------------------------------------------------------
bool PathMatching::tryToMatchOnPath_(const Pose2D & vehicle_pose,
                                     const Twist2D & vehicle_twist)
{

  double vehicle_speed =vehicle_twist.linearSpeeds.x();

  if(matched_points_.empty())
  {

    matched_points_ = match(*path_,
                            vehicle_pose,
                            vehicle_speed,
                            prediction_time_horizon_,
                            maximal_research_radius_);

  }
  else
  {

    matched_points_ = match(*path_,
                            vehicle_pose,
                            vehicle_speed,
                            matched_points_[tracked_matched_point_index_],
                            2,
                            prediction_time_horizon_,
                            maximal_research_radius_);

  }

  if(!matched_points_.empty())
  {
    tracked_matched_point_index_ = bestMatchedPointIndex(matched_points_,vehicle_speed);
  }

  diagnostics_.updateMatchingStatus(!matched_points_.empty());
  return !matched_points_.empty();
}


//-----------------------------------------------------------------------------
void PathMatching::displayResults_(const Pose2D & vehicle_pose)
{

  display_.deleteMarkers();
  display_.displayPathMarkers(vehicle_pose);

  if(!matched_points_.empty())
  {
    const auto & section= path_->getSection(matched_points_[tracked_matched_point_index_].sectionIndex);
    const auto & curve = section.getCurve(matched_points_[tracked_matched_point_index_].curveIndex);
    display_.displayCurveMarkers(curve);
  }

  display_.trigger();
}

//-----------------------------------------------------------------------------
void PathMatching::timerCallback(const ros::TimerEvent & event)
{
  diagnostic_pub_.publish(event.current_real,diagnostics_.getReport());
  tf_.publish(event.current_real);
}


}
