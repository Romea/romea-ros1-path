//romea
#include "on_the_fly_path_matching.hpp"
#include <romea_common/math/EulerAngles.hpp>
#include <romea_common_utils/conversions/transform_conversions.hpp>

namespace  {
const double DEFAULT_MINIMAL_DISTANCE_BETWEEN_TWO_POINTS = 0.2;
const double DEFAULT_MINIMAL_VEHICLE_SPEED_TO_INSERT_POINT = 0.1;

inline Eigen::Vector2d position2d(const geometry_msgs::Point & p)
{
  return Eigen::Vector2d(p.x,p.y);
}

inline double speed2d(const geometry_msgs::Vector3 & v)
{
  return std::sqrt(v.x*v.x + v.y*v.y);
}


}

namespace romea {

//-----------------------------------------------------------------------------
OnTheFlyPathMatching::OnTheFlyPathMatching():
  display_(),
  path_(nullptr),
  matched_point_(),
  leader_odom_sub_(),
  leader_odom_callback_queue_()
{  
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::init(ros::NodeHandle
                               &nh, ros::NodeHandle &private_nh)
{
  configureMaximalResearshRadius_(private_nh);
  configureInterpolationWindowLength_(private_nh);
  path_=std::make_unique<PathSection2D>(interpolation_window_length_);
  configurePredictionTimeHorizon_(private_nh);
  configureRecorder_(private_nh);
  configureMatchingInfoPublisher_(nh);
  configureLeaderOdomSubscriber_(nh);
  configureOdomSubscriber_(nh);
  display_.init(private_nh);
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::configureLeaderOdomSubscriber_(ros::NodeHandle &nh)
{
  auto options = ros::SubscribeOptions::create<nav_msgs::Odometry>("leader_odom",
                                                              10,
                                                              boost::bind(&OnTheFlyPathMatching::processLeaderOdom_, this,  _1),
                                                              ros::VoidPtr(),
                                                              &leader_odom_callback_queue_);

  leader_odom_sub_ = nh.subscribe(options);

  ROS_ERROR_STREAM(" path matchin leader topic " << leader_odom_sub_.getTopic());

}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::configureRecorder_(ros::NodeHandle &private_nh)
{
  minimalDistanceBetweenTwoPoints_=private_nh.param("minimal_distance_between_two_points",
                                                    DEFAULT_MINIMAL_DISTANCE_BETWEEN_TWO_POINTS);


  minimalVehicleSpeedToInsertPoint_=private_nh.param("minimal_vehicle_speed_to_insert_point_",
                                                     DEFAULT_MINIMAL_VEHICLE_SPEED_TO_INSERT_POINT);

}


//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::reset()
{
  display_.reset();
  matched_point_.reset();
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::processLeaderOdom_(const nav_msgs::Odometry::ConstPtr &msg)
{

  ROS_ERROR_STREAM("OnTheFlyPathMatching::processLeaderOdom_");

  Eigen::Vector2d position = position2d(msg->pose.pose.position);
  static Eigen::Vector2d previous_position = position;

  double distance = (position-previous_position).norm();
  double vehicle_speed = speed2d(msg->twist.twist.linear);

  if(distance > minimalDistanceBetweenTwoPoints_ &&
     vehicle_speed > minimalVehicleSpeedToInsertPoint_)
  {
    ROS_ERROR_STREAM(position.transpose());
    path_->addWayPoint(PathWayPoint2D(position));
    display_.addPoint(position);
  }

}


//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::processOdom_(const nav_msgs::Odometry::ConstPtr &msg)
{


  ROS_ERROR_STREAM(" OnTheFlyPathMatching::processOdom_ ");

  leader_odom_callback_queue_.callAvailable();

  PoseAndTwist3D enuPoseAndBodyTwist3D;
  toRomea(msg->pose,enuPoseAndBodyTwist3D.pose);
  toRomea(msg->twist,enuPoseAndBodyTwist3D.twist);
  romea::Pose2D vehicle_pose = toPose2D(enuPoseAndBodyTwist3D.pose);
  romea::Twist2D vehicle_twist = toTwist2D(enuPoseAndBodyTwist3D.twist);

  diagnostics_.updateOdomRate(romea::toRomeaDuration(msg->header.stamp));


  if(path_->size()>0)
  {
    ROS_ERROR_STREAM(" path_.getNumberOfPoints() "<< path_->size());

    if(tryToMatchOnPath_(vehicle_pose,vehicle_twist))
    {

      std::cout << *matched_point_ << std::endl;
//      matched_point_->futureCurvature = computeFutureCurvature_(msg->twist.twist.linear.x);
      match_pub_.publish(romea::toRosMsg(msg->header.stamp,
                                         *matched_point_,
                                         path_->getLength(),
                                         vehicle_twist));
    }
    else
    {

      ROS_ERROR_STREAM(" fake matched point ");

      PathMatchedPoint2D fake_matched_point;
      Eigen::Vector2d first_position(path_->getX()[0],path_->getX()[1]);
      Eigen::Vector2d vehicle_position(vehicle_pose.position.x(),vehicle_pose.position.y());
      Eigen::Vector2d fake_segment_direction = vehicle_position -first_position;

      ROS_ERROR_STREAM(" fake matched point ");

      fake_matched_point.pathPosture.course=std::atan2(fake_segment_direction.y(),fake_segment_direction.x());
      fake_matched_point.pathPosture.position = vehicle_position;
      fake_matched_point.pathPosture.curvature =0;
      fake_matched_point.pathPosture.dotCurvature =0;

      ROS_ERROR_STREAM(" fake matched point ");

      fake_matched_point.frenetPose.lateralDeviation=0;
      fake_matched_point.frenetPose.courseDeviation=betweenMinusPiAndPi(vehicle_pose.yaw-fake_matched_point.pathPosture.course);
      fake_matched_point.frenetPose.curvilinearAbscissa = - fake_segment_direction.norm();
//      fake_matched_point.nearestPointIndex=0;


      ROS_ERROR_STREAM(" fake matched point ");

      match_pub_.publish(romea::toRosMsg(msg->header.stamp,
                                         fake_matched_point,
                                         path_->getLength(),
                                         vehicle_twist));

    }
  }

  displayResults_(vehicle_pose);
}

//-----------------------------------------------------------------------------
bool OnTheFlyPathMatching::tryToMatchOnPath_(const Pose2D & vehicle_pose,
                                            const Twist2D & vehicle_twist)
{

  if(path_->getLength()<2)
  {
    return false;
  }

  double vehicle_speed =vehicle_twist.linearSpeeds.x();
  if(matched_point_.is_initialized())
  {
    matched_point_ = match(*path_,
                           vehicle_pose,
                           vehicle_speed,
                           *matched_point_,
                           10,
                           prediction_time_horizon_,
                           maximal_research_radius_);
  }
  else
  {
    matched_point_ = match(*path_,
                           vehicle_pose,
                           vehicle_speed,
                           prediction_time_horizon_,
                           maximal_research_radius_);
  }

  diagnostics_.updateMatchingStatus(matched_point_.is_initialized());
  return matched_point_.is_initialized();
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::displayResults_(const Pose2D & vehicle_pose)
{

  display_.deleteMarkers();
  display_.displayPathMarkers(vehicle_pose);

  if(matched_point_.is_initialized())
  {
    const auto & curve = path_->getCurve(matched_point_->curveIndex);
    display_.displayCurveMarkers(curve);
  }

  display_.trigger();
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::timerCallback(const ros::TimerEvent &event)
{
    diagnostic_pub_.publish(event.current_real,diagnostics_.getReport());
}


}
