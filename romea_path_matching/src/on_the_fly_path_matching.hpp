#ifndef romea_OnTheFlyPathMatching_HPP
#define romea_OnTheFlyPathMatching_HPP

//romea
#include <romea_path/PathSection2D.hpp>
#include "path_matching_base.hpp"
#include "on_the_fly_path_matching_display.hpp"
#include "on_the_fly_path_matching_diagnostic.hpp"

//ros
#include <ros/callback_queue.h>

namespace romea {

class OnTheFlyPathMatching : public PathMatchingBase
{

public :

  OnTheFlyPathMatching();

  virtual ~OnTheFlyPathMatching() =default;

  virtual void init(ros::NodeHandle & nh, ros::NodeHandle & private_nh)override;

  virtual void timerCallback(const ros::TimerEvent & event)override;

  virtual void reset()override;

protected:

  void configureRecorder_(ros::NodeHandle &private_nh);

  void configureLeaderOdomSubscriber_(ros::NodeHandle &nh);

  virtual void processOdom_(const nav_msgs::Odometry::ConstPtr &msg)override;

  void processLeaderOdom_(const nav_msgs::Odometry::ConstPtr &msg);

  bool tryToMatchOnPath_(const Pose2D & vehicle_pose, const Twist2D &vehicle_twist);

  void displayResults_(const Pose2D & vehicle_pose);

protected :


  OnTheFlyPathMatchingDisplay display_;
  OnTheFlyPathMatchingDiagnostic diagnostics_;

  std::unique_ptr<PathSection2D> path_;
  std::optional<romea::PathMatchedPoint2D> matched_point_;

  ros::Subscriber leader_odom_sub_;
  ros::CallbackQueue leader_odom_callback_queue_;

  double minimalDistanceBetweenTwoPoints_;
  double minimalVehicleSpeedToInsertPoint_;

};

}
#endif
