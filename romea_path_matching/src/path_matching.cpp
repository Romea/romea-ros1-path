//romea
#include "path_matching.hpp"
#include <ros/PathMatchingInfoConversions.hpp>
#include <ros/conversions/TransformConversions.hpp>
#include <ros/conversions/PoseAndTwist3DConversions.hpp>
#include <ros/RvizDisplay.hpp>

//std
#include <fstream>

//ros
#include <eigen_conversions/eigen_msg.h>

namespace {
const double DEFAULT_MAXIMAL_REASEARCH_RADIUS =10;
const double DEFAULT_INTERPOLATION_WINDOW_LENGTH =3;
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
  tf_map_to_path_(),
  tf_world_to_path_(),
  tf_world_to_map_(),
  tf_listener_(),
  tf_broadcaster_(),
  tf_world_to_path_msg_(),
  display_(true),
  rviz_util_("path","matching"),
  path3d_(),
  interpolatedPath3d_(30),
  diagnostics_()
{  
}

//-----------------------------------------------------------------------------
void PathMatching::init(ros::NodeHandle nh, ros::NodeHandle private_nh)
{

  ROS_INFO("Init PathMatching");


  //maximal_researh_radius
  double maximal_researh_radius;
  private_nh.param("maximal_researh_radius",maximal_researh_radius,DEFAULT_MAXIMAL_REASEARCH_RADIUS);
  path_matching_.setMaximalResearchRadius(maximal_researh_radius);

  //maximal_researh_radius
  double interpolation_window_length;
  private_nh.param("interpolation_window_length",interpolation_window_length,DEFAULT_INTERPOLATION_WINDOW_LENGTH);
  path_.setInterpolationWindowLength(interpolation_window_length);

  tf_world_to_path_msg_.header.frame_id = "world";
  tf_world_to_path_msg_.child_frame_id = "path";
  tf_map_to_path_msg_.header.frame_id = "path";
  tf_map_to_path_msg_.child_frame_id = "map";

  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("filtered_odom", 10, &PathMatching::processOdom_,this);
  match_pub_ = nh.advertise<romea_path_msgs::PathMatchingInfo2D>("path_matching_info",1);

  private_nh.param("display",display_,false);
  initDisplay_();
}

//-----------------------------------------------------------------------------
void PathMatching::reset()
{
  matched_point_.reset();
}

//-----------------------------------------------------------------------------
void PathMatching::loadPath(const std::string & filename, bool revert)
{
  ROS_INFO_STREAM("Load PathMatching path '" << filename << "'");

  std::ifstream file(filename);
  diagnostics_.updatePathStatus(filename,file.is_open());

  if(file.is_open())
  {

    std::string header;
    file >> header;

    if(header.compare("WGS84")==0)
    {
      double reference_latitude;
      double reference_longitude;
      double reference_altitude;

      file >> reference_latitude >> reference_longitude >> reference_altitude;

      romea::GeodeticCoordinates anchor(reference_latitude/180.*M_PI,
                                        reference_longitude/180.*M_PI,
                                        reference_altitude);

      tf_world_to_path_= romea::toRosTransform(ENUConverter(anchor));
    }
    else if(header.compare("ENU")==0 || header.compare("PIXEL")==0)
    {
      tf_world_to_path_.setIdentity();
    }
    else
    {
      throw(std::runtime_error("Failed to read path file "+ filename));
    }

    tf_world_to_path_msg_.header.stamp = ros::Time::now();
    tf::transformTFToMsg(tf_world_to_path_,tf_world_to_path_msg_.transform);


    romea::VectorOfEigenVector2d points;
    points.reserve(100000);
    path3d_.reserve(100000);

    double x,y;
    while(!file.eof())
    {
      file >> x >> y;
      points.emplace_back(x,y);
      path3d_.emplace_back(x,y,0);
    }

    if(revert)
    {
      std::reverse(std::begin(points), std::end(points));
      std::reverse(std::begin(path3d_), std::end(path3d_));
    }

    path_.load(points);
  }
  else
  {
    throw(std::runtime_error("Failed to open path file "+ filename));
  }

}

//-----------------------------------------------------------------------------
void PathMatching::publishTf(const ros::TimerEvent & /*event*/)
{
  diagnostics_.publish();
  tf_broadcaster_.sendTransform(tf_world_to_path_msg_);
}


//-----------------------------------------------------------------------------
void PathMatching::processOdom_(const nav_msgs::Odometry::ConstPtr &msg)
{

  //std::cout << msg->header.stamp <<"process odom"<<std::endl;

  if(path_.isLoaded())
  {

    std::string map_frame_id, frame_child_id;
    romea::PoseAndTwist3D::Stamped  enuPoseAndBodyTwist3D=romea::toRomea(*msg,map_frame_id,frame_child_id);
    diagnostics_.updateOdomRate(romea::toRomeaDuration(msg->header.stamp));
    //std::cout <<"frame_id "<< map_frame_id <<" "<< frame_child_id<<std::endl;

    if(tryToEvaluteMapToPathTransformation_(msg->header.stamp,map_frame_id))
    {

      romea::Pose2D vehicle_pose = (tf_map_to_path_*enuPoseAndBodyTwist3D.data.getPose()).toPose2D();
      romea::Twist2D vehicle_twist = enuPoseAndBodyTwist3D.data.getTwist().toTwist2D();

      if(tryToMatchOnPath_(vehicle_pose))
      {

        std::cout << *matched_point_ << std::endl;

        double future_curvature = path_matching_.computeFutureCurvature(path_,
                                                                        *matched_point_,
                                                                        msg->twist.twist.linear.x);
        match_pub_.publish(romea::toRosMsg(msg->header.stamp,
                                           *matched_point_,
                                           path_.getLength(),
                                           future_curvature,
                                           vehicle_twist));


      }

      displayResults_(vehicle_pose);
    }
  }
}


//-----------------------------------------------------------------------------
bool PathMatching::tryToEvaluteMapToPathTransformation_(const ros::Time &stamp,
                                                        const std::string & map_frame_id)
{
  try{

#warning anti date can cause trouble if map reference frame change
    tf_listener_.lookupTransform("world",map_frame_id,stamp - ros::Duration(0.2),tf_world_to_map_);
    tf::transformTFToEigen((tf_world_to_map_*tf_world_to_path_.inverse()).inverse(),tf_map_to_path_);
    diagnostics_.updateLookupTransformStatus(true);

//    std::cout << " tf_map_to_path_ "<< std::endl;
//    std::cout <<  tf_map_to_path_.matrix() << std::endl;

    tf_map_to_path_msg_.header.stamp = stamp;
    tf::transformEigenToMsg(tf_map_to_path_,tf_map_to_path_msg_.transform);
    tf_broadcaster_.sendTransform(tf_world_to_path_msg_);
//    tf_broadcaster_.sendTransform(tf_map_to_path_msg_);
// problem casse ote map to world tf

    return true;
  }
  catch (tf::TransformException ex)
  {
    std::cout << " catch " << std::endl;
    std::cout <<  ex.what() << std::endl;
    diagnostics_.updateLookupTransformStatus(false);
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


//-----------------------------------------------------------------------------
void PathMatching::initDisplay_()
{
  if(display_)
  {
    rviz_util_.loadMarkerPub();
    rviz_util_.deleteAllMarkers();
    rviz_util_.enableBatchPublishing();
  }
}


//-----------------------------------------------------------------------------
void PathMatching::displayInterpolatedPath_()
{
  const romea::PathCurve2D & pathCurve = path_.getCurves()[matched_point_->getNearestPointIndex()];
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
void PathMatching::displayResults_(const Pose2D & vehicle_pose)
{
  if(display_)
  {
    rviz_util_.deleteAllMarkers();
    rviz_util_.publishSpheres(path3d_,rviz_visual_tools::WHITE,rviz_visual_tools::XXLARGE);
    romea::publish(rviz_util_,vehicle_pose,rviz_visual_tools::GREEN);

    if(matched_point_.is_initialized())
    {
      displayInterpolatedPath_();
    }

    rviz_util_.trigger();
  }

}


}
