//romea
#include "path_matching.hpp"
#include <ros/PathMatchingInfoConversions.hpp>
#include <ros/TransformConversions.hpp>
#include <ros/Pose2DRvizDisplay.hpp>
#include <ros/OdomConversions.hpp>

//std
#include <fstream>

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
  rviz_util_("map","matching"),
  path3d_(),
  interpolatedPath3d_(30),
  diagnostics_()
{  
}

//-----------------------------------------------------------------------------
bool PathMatching::init(ros::NodeHandle nh, ros::NodeHandle private_nh)
{

  //init visual tools
  if(!private_nh.getParam("display",display_))
  {
    ROS_ERROR("Failed to read rviz display status from launch file ");
  }

  if(display_)
  {
    rviz_util_.loadMarkerPub();
    rviz_util_.deleteAllMarkers();
    rviz_util_.enableBatchPublishing();
  }

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

  odom_sub_ = nh.subscribe<nav_msgs::Odometry>(nh.resolveName("filtered_odom"), 10, &PathMatching::processOdom_,this);
  match_pub_ = nh.advertise<romea_path_msgs::PathMatchingInfo2D>(nh.resolveName("path_matching_info"),1);
  return true;
}

//-----------------------------------------------------------------------------
void PathMatching::reset()
{
  matched_point_.reset();
}

//-----------------------------------------------------------------------------
bool PathMatching::loadPath(const std::string & filename)
{

  std::ifstream file(filename);
  std::cout <<" filename "<< filename << std::endl;
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


      // publish
      tf_world_to_path_= romea::toRosTransform(ENUConverter(anchor));
    }
    else
    {
      // mettre idenity
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
      points.emplace_back(Eigen::Vector2d(x,y));
      path3d_.emplace_back(Eigen::Vector3d(x,y,0));
    }

    path_.load(points);

  }

  diagnostics_.updatePathStatus(filename,file.is_open());
  return file.is_open();
}

//-----------------------------------------------------------------------------
void PathMatching::publishTf(const ros::TimerEvent & /*event*/)
{
  tf_broadcaster_.sendTransform(tf_world_to_path_msg_);
}

//-----------------------------------------------------------------------------
void PathMatching::processOdom_(const nav_msgs::Odometry::ConstPtr &msg)
{

  if(path_.isLoaded())
  {
    std::string frame_id, frame_child_id;
    romea::PoseAndTwist3D::Stamped  enuPoseAndBodyTwist3D=romea::toRomea(*msg,frame_id,frame_child_id);
    diagnostics_.updateOdomRate(romea::toRomeaDuration(msg->header.stamp));

    std::cout << msg->header.stamp <<"process odom"<<std::endl;

    try{

#warning anti date can cause trouble if map reference frame change
      tf_listener_.lookupTransform("world",frame_id,msg->header.stamp-ros::Duration(0.2),tf_world_to_map_);
      tf::transformTFToEigen((tf_world_to_map_*tf_world_to_path_.inverse()).inverse(),tf_map_to_path_);

      romea::Pose2D vehiclePose2D = (tf_map_to_path_*enuPoseAndBodyTwist3D.data.getPose()).toPose2D();
      romea::Twist2D vehicleTwist2D = enuPoseAndBodyTwist3D.data.getTwist().toTwist2D();

      diagnostics_.updateLookupTransformStatus(true);

      if(matched_point_)
      {
        matched_point_ = path_matching_.match(path_,vehiclePose2D,*matched_point_,10);
      }
      else
      {
        matched_point_ = path_matching_.match(path_,vehiclePose2D);
      }


      if(matched_point_)
      {

        double future_curvature = path_matching_.computeFutureCurvature(path_,
                                                                        *matched_point_,
                                                                        msg->twist.twist.linear.x);
        match_pub_.publish(romea::toROSMsg(msg->header.stamp,
                                           *matched_point_,
                                           path_.getLength(),
                                           future_curvature,
                                           vehicleTwist2D));


      }
      else
      {
      }

      diagnostics_.updateMatchingStatus(matched_point_.is_initialized());

      if(display_)
      {
        rviz_util_.deleteAllMarkers();
        rviz_util_.publishSpheres(path3d_,rviz_visual_tools::WHITE,rviz_visual_tools::XXLARGE);

        const romea::PathCurve2D pathCurve = path_matching_.getInterpolatedPath();
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
        romea::publish(rviz_util_,vehiclePose2D,rviz_visual_tools::GREEN);

        rviz_util_.triggerBatchPublish();
      }
    }
    catch (tf::TransformException ex)
    {
      std::cout << " catch " << std::endl;
      std::cout <<  ex.what() << std::endl;
      diagnostics_.updateLookupTransformStatus(false);
    }
  }
}

}
