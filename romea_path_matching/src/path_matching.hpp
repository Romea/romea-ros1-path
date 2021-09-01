#ifndef PathMatching_HPP
#define PathMatching_HPP


#include "path_matching_base.hpp"
#include "path_matching_display.hpp"
#include "path_matching_diagnostic.hpp"
#include "path_matching_tf.hpp"



namespace romea {

class PathMatching : public PathMatchingBase
{

public :

  PathMatching();

  virtual ~PathMatching()=default;

  virtual void init(ros::NodeHandle & nh, ros::NodeHandle & private_nh)override;

  virtual void timerCallback(const ros::TimerEvent & event)override;

  void loadPath(const std::string & filename);

  virtual void reset() override;

protected:

  virtual void processOdom_(const nav_msgs::Odometry::ConstPtr &msg)override;

  bool tryToEvaluteMapToPathTransformation_(const ros::Time & stamp,const std::string & map_frame_id);

  bool tryToMatchOnPath_(const Pose2D & vehicle_pose, const Twist2D & vehicle_twist);

  void displayResults_(const Pose2D &vehicle_pose);


protected:

  PathMatchingTf tf_;
  PathMatchingDisplay display_;
  PathMatchingDiagnostic diagnostics_;

  std::unique_ptr<Path2D> path_;
  std::vector<PathMatchedPoint2D> matched_points_;
  size_t tracked_matched_point_index_;

};

}
#endif
