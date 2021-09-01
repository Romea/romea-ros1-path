#ifndef romea_PathMatchingBase_HPP
#define romea_PathMatchingBase_HPP

//ros
#include <nav_msgs/Odometry.h>

//romea
#include <romea_path/PathMatching2D.hpp>
#include <romea_path_utils/path_matching_info_conversions.hpp>
#include <romea_localisation_utils/conversions/pose_and_twist3d_conversions.hpp>
#include <romea_common_utils/publishers/diagnostic_publisher.hpp>

namespace romea {

class PathMatchingBase
{

public :

  PathMatchingBase();

  virtual ~PathMatchingBase()=default;

  virtual void init(ros::NodeHandle & nh, ros::NodeHandle & private_nh)=0;

  virtual void timerCallback(const ros::TimerEvent & event)=0;

  virtual void reset() =0;

protected:

  virtual void processOdom_(const nav_msgs::Odometry::ConstPtr &msg)=0;

  void configureMaximalResearshRadius_(ros::NodeHandle & private_nh);

  void configureInterpolationWindowLength_(ros::NodeHandle & private_nh);

  void configurePredictionTimeHorizon_(ros::NodeHandle & private_nh);

  void configureMatchingInfoPublisher_(ros::NodeHandle & nh);

  void configureDiagnosticPublisher_(ros::NodeHandle & nh);

  void configureOdomSubscriber_(ros::NodeHandle & nh);

protected:

  double prediction_time_horizon_;
  double maximal_research_radius_;
  double interpolation_window_length_;

  ros::Subscriber odom_sub_;
  ros::Publisher match_pub_;
  DiagnosticPublisher<DiagnosticReport> diagnostic_pub_;

};

}
#endif
