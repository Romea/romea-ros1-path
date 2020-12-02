//ros
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

//romea
#include "path_matching.hpp"
#include <romea_lifecycle/LifecycleNodelet.hpp>
#include <romea_path_msgs/LoadPath.h>
namespace romea {


class PathMatchingNodelet : public LifecycleNodelet
{
public:

  PathMatchingNodelet();

  virtual ~PathMatchingNodelet()=default;

  virtual void onConfigure()override;

  virtual void onActivate()override;

  virtual void onDeactivate()override;

private :

  bool loadPathCallback_(romea_path_msgs::LoadPath::Request & request,
                         romea_path_msgs::LoadPath::Response &);

private :

  ros::Timer timer_;
  ros::ServiceServer load_path_service_;
  PathMatching path_matching_;
};

}
