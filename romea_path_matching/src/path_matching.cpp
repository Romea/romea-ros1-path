//romea
#include "path_matching.hpp"
#include <romea_common/geodesy/ENUConverter.hpp>
#include <romea_path_utils/PathMatchingInfoConversions.hpp>
#include <romea_common_utils/conversions/TransformConversions.hpp>
#include <romea_localisation_utils/conversions/PoseAndTwist3DConversions.hpp>
#include <romea_localisation_utils/RvizDisplay.hpp>

//std
#include <fstream>

//ros
#include <eigen_conversions/eigen_msg.h>

namespace {
const double DEFAULT_MAXIMAL_REASEARCH_RADIUS =10;
const double DEFAULT_INTERPOLATION_WINDOW_LENGTH =3;
const double DEFAULT_PREDICTION_TIME_HORIZON=0.5;
}

namespace romea {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatching::PathMatching():
  path_(),
  path_matching_(),
  matched_point_(),
  odom_sub_(),
  match_pub_(),
  diagnostic_pub_(),
  map_to_path_(),
  world_to_path_(),
  world_to_map_(),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  tf_broadcaster_(),
  tf_world_to_path_msg_(),
  rviz_(),
  diagnostics_()
{  
}

//-----------------------------------------------------------------------------
void PathMatching::init(ros::NodeHandle nh, ros::NodeHandle private_nh)
{

  //  ROS_INFO("Init PathMatching");

  //maximal_researh_radius
  double maximal_researh_radius;
  private_nh.param("maximal_researh_radius",
                   maximal_researh_radius,
                   DEFAULT_MAXIMAL_REASEARCH_RADIUS);
  path_matching_.setMaximalResearchRadius(maximal_researh_radius);

  //maximal_researh_radius
  double interpolation_window_length;
  private_nh.param("interpolation_window_length",
                   interpolation_window_length,
                   DEFAULT_INTERPOLATION_WINDOW_LENGTH);
  path_.setInterpolationWindowLength(interpolation_window_length);


  private_nh.param("prediction_time_horizon",
                   prediction_time_horizon_,
                   DEFAULT_PREDICTION_TIME_HORIZON);

  tf_world_to_path_msg_.header.frame_id = "world";
  tf_world_to_path_msg_.child_frame_id = "path";

  if(private_nh.param("display",false))
  {
    rviz_.enable();
  }

  diagnostic_pub_.init(nh,"path_matching",1.0);
  match_pub_ = nh.advertise<romea_path_msgs::PathMatchingInfo2D>("path_matching_info",1);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("filtered_odom", 10, &PathMatching::processOdom_,this);

}

//-----------------------------------------------------------------------------
void PathMatching::reset()
{
  matched_point_.reset();
}

//-----------------------------------------------------------------------------
void PathMatching::loadPath(const std::string & filename, bool revert)
{
  //  ROS_INFO_STREAM("Load PathMatching path '" << filename << "'");
  std::cout <<" PathMatching !!!!!! " << filename << std::endl;
  std::ifstream file(filename);
  diagnostics_.setPathFilename(filename);

  if(file.is_open())
  {

    std::string header;
    file >> header;

//    ENUConverter enuConverter;
    if(header.compare("WGS84")==0)
    {
      double reference_latitude;
      double reference_longitude;
      double reference_altitude;

      file >> reference_latitude >> reference_longitude >> reference_altitude;

      romea::GeodeticCoordinates anchor(reference_latitude/180.*M_PI,
                                        reference_longitude/180.*M_PI,
                                        reference_altitude);

//      enuConverter.setAnchor(anchor);
      world_to_path_= ENUConverter(anchor).getEnuToEcefTransform();
    }
    else if(header.compare("ENU")==0 || header.compare("PIXEL")==0)
    {
      world_to_path_= Eigen::Affine3d::Identity();
    }
    else
    {
      throw(std::runtime_error("Failed to read path file "+ filename));
    }

    //    tf_world_to_path_msg_.header.stamp = ros::Time::now();
    //    toRosTransformMsg(world_to_path_,tf_world_to_path_msg_.transform);

    romea::VectorOfEigenVector2d points;
    points.reserve(100000);

    double x,y;
    while(!file.eof())
    {
      file >> x >> y;
      points.emplace_back(x,y);
//      std::cout << std::setprecision(10)<< enuConverter.toWGS84(x,y,0).getLatitude()*180/M_PI <<" ";
//      std::cout << enuConverter.toWGS84(x,y,0).getLongitude()*180/M_PI <<std::endl;
    }

    if(revert)
    {
      std::reverse(std::begin(points), std::end(points));
    }

    path_.load(points);
    rviz_.configure(path_);
  }
  else
  {
    throw(std::runtime_error("Failed to open path file "+ filename));
  }

}

//-----------------------------------------------------------------------------
void PathMatching::publishTf(const ros::TimerEvent & event)
{
  diagnostics_.updatePathStatus(path_.isLoaded());
  diagnostic_pub_.publish(event.current_real,diagnostics_.getReport());

  tf_world_to_path_msg_.header.stamp=event.current_real;
  toRosTransformMsg(world_to_path_,tf_world_to_path_msg_.transform);
  tf_broadcaster_.sendTransform(tf_world_to_path_msg_);
}


//-----------------------------------------------------------------------------
void PathMatching::processOdom_(const nav_msgs::Odometry::ConstPtr &msg)
{

//  std::cout << msg->header.stamp <<"process odom"<<std::endl;

  if(path_.isLoaded())
  {
    PoseAndTwist3D enuPoseAndBodyTwist3D;
    toRomea(msg->pose,enuPoseAndBodyTwist3D.pose);
    toRomea(msg->twist,enuPoseAndBodyTwist3D.twist);
    diagnostics_.updateOdomRate(romea::toRomeaDuration(msg->header.stamp));

    if(tryToEvaluteMapToPathTransformation_(msg->header.stamp,msg->header.frame_id))
    {
      romea::Pose2D vehicle_pose = toPose2D(map_to_path_*enuPoseAndBodyTwist3D.pose);
      romea::Twist2D vehicle_twist = toTwist2D(enuPoseAndBodyTwist3D.twist);

      if(tryToMatchOnPath_(vehicle_pose))
      {

//        std::cout << *matched_point_ << std::endl;

        double future_curvature = path_matching_.computeFutureCurvature(path_,
                                                                        *matched_point_,
                                                                        msg->twist.twist.linear.x,
                                                                        prediction_time_horizon_);

//        std::cout <<" future_curvature "<< future_curvature << std::endl;
        match_pub_.publish(romea::toRosMsg(msg->header.stamp,
                                           *matched_point_,
                                           path_.getLength(),
                                           future_curvature,
                                           vehicle_twist));


        rviz_.display(vehicle_pose,&path_.getCurves()[matched_point_->nearestPointIndex]);

      }
      else
      {
        rviz_.display(vehicle_pose,nullptr);
      }
    }
  }
}



//-----------------------------------------------------------------------------
bool PathMatching::tryToEvaluteMapToPathTransformation_(const ros::Time &stamp,
                                                        const std::string & map_frame_id)
{
  try
  {
#warning anti date can cause trouble if map reference frame change
    geometry_msgs::TransformStamped transformStamped;
    transformStamped=tf_buffer_.lookupTransform("world",map_frame_id,stamp - ros::Duration(0.2));
    world_to_map_ = tf2::transformToEigen(transformStamped);
    map_to_path_ = tf2::transformToEigen(transformStamped).inverse()*world_to_map_;
    diagnostics_.updateLookupTransformStatus(true);
    return true;
  }
  catch (tf2::TransformException ex)
  {
    std::cout << " catch " << std::endl;
    std::cout <<  ex.what() << std::endl;
    diagnostics_.updateLookupTransformStatus(false);
    diagnostics_.updateMatchingStatus(false);
    return false;
  }
}



//-----------------------------------------------------------------------------
bool PathMatching::tryToMatchOnPath_(const Pose2D & vehicle_pose)
{
  if(matched_point_.is_initialized())
  {
    matched_point_ = path_matching_.match(path_,vehicle_pose,*matched_point_,10);
  }
  else
  {
    matched_point_ = path_matching_.match(path_,vehicle_pose);
  }

  diagnostics_.updateMatchingStatus(matched_point_.is_initialized());
  return matched_point_.is_initialized();
}

}
