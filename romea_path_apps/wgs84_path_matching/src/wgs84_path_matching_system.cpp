//romea
#include "wgs84_path_matching_system.hpp"
#include <ros_path_utils.hpp>
#include <ros_transform_util.hpp>
#include <ros_localisation_util.hpp>

//std
#include <fstream>

const double DEFAULT_MAXIMAL_REASEARCH_RADIUS =10;
const double DEFAULT_INTERPOLATION_WINDOW_LENGTH =10;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
WGS84PathMatchingSystem::WGS84PathMatchingSystem(ros::NodeHandle node, ros::NodeHandle private_nh):
  wgs84_path_(),
  enu_path_matching_(),
  display_(false),
  rviz_util_("map","communications")
//  diagnostics_()
{

  //init visual tools
  if(!private_nh.getParam("/r2hl/display",display_))
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
  enu_path_matching_.setMaximalResearchRadius(maximal_researh_radius);

  //maximal_researh_radius
  double interpolation_window_length;
  private_nh.param("interpolation_window_length",interpolation_window_length,DEFAULT_INTERPOLATION_WINDOW_LENGTH);
  enu_path_matching_.setInterpolationWindowLength(interpolation_window_length);


  tf_world_to_path_msg_.header.frame_id = "world";
  tf_world_to_path_msg_.child_frame_id = "path";

  odom_sub_ = node.subscribe<nav_msgs::Odometry>("/odometry", 10, &WGS84PathMatchingSystem::processOdom,this);
  match_pub_ = node.advertise<romea_path_msgs::PathMatchedPoint2DStamped>("/path_matched_pooint",1);
  timer_ = node.createTimer(ros::Rate(1), &WGS84PathMatchingSystem::publishTf_, this);


}

//-----------------------------------------------------------------------------
void WGS84PathMatchingSystem::loadPath_(const std::string & path_filename)
{

  enu_matched_point_.reset();

  std::ifstream file(path_filename);
  if(file.is_open())
  {
    double reference_latitude;
    double reference_longitude;
    double reference_altitude;

    file >> reference_latitude >> reference_longitude >> reference_altitude;

    romea::GeodeticCoordinates anchor(reference_latitude/180.*M_PI,
                                      reference_longitude/180.*M_PI,
                                      reference_altitude);

    wgs84_path_.enuCoordinateSystems.setAnchor(anchor);
    tf_world_to_path_= romea::toRosTransform(wgs84_path_.enuCoordinateSystems);

    tf_world_to_path_msg_.header.stamp = ros::Time::now();
    tf::transformTFToMsg(tf_world_to_path_,tf_world_to_path_msg_.transform);

    size_t numberOfPoints;
    file >> numberOfPoints;

    Eigen::Vector2d point;
    romea::VectorOfEigenVector<Eigen::Vector2d> points;

    while(!file.eof())
    {
      file >> point[0] >> point[1];
      points.push_back(point);
    }

    wgs84_path_.enuPath.load(points);
  }
}

//-----------------------------------------------------------------------------
void WGS84PathMatchingSystem::publishTf_(const ros::TimerEvent & event)
{

  tf_broadcaster_.sendTransform(tf_world_to_path_msg_);
}

//-----------------------------------------------------------------------------
void WGS84PathMatchingSystem::processOdom(const nav_msgs::Odometry::ConstPtr &msg)
{

  std::string frame_id, frame_child_id;
  romea::PoseAndTwist3D::Stamped  enuPoseAndBodyTwist3D=romea::toRomea(*msg,frame_id,frame_child_id);

  try{

    tf_listener_.lookupTransform(frame_id,"/world",msg->header.stamp,tf_world_to_map_);
    tf::transformTFToEigen(tf_world_to_map_.inverseTimes(tf_world_to_path_),tf_map_to_path_);
    romea::Pose2D vehiclePose2D = (tf_map_to_path_*enuPoseAndBodyTwist3D.data.getPose()).toPose2D();

    if(enu_matched_point_)
    {
      enu_matched_point_ = enu_path_matching_.match(wgs84_path_.enuPath,
                                                    vehiclePose2D,
                                                    *enu_matched_point_,
                                                    10);
    }
    else
    {
      enu_matched_point_ = enu_path_matching_.match(wgs84_path_.enuPath,vehiclePose2D);
    }


    if(enu_matched_point_)
    {
      match_pub_.publish(romea::toROSMsg(msg->header.stamp,*enu_matched_point_));
    }
    else
    {
      //TODO diag
    }
  }
  catch (tf::TransformException ex)
  {
    //TODO diag
  }

}


