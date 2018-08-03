//ros
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

//romea
#include "path_matching.hpp"
#include <romea_fsm_msgs/FSMService.h>

namespace romea {


class PathMatchingNodelet : public nodelet::Nodelet
{
public:

  PathMatchingNodelet();

  virtual ~PathMatchingNodelet();

  virtual void onInit()override;

protected :

  bool serviceCallback_(romea_fsm_msgs::FSMService::Request  &request,
                        romea_fsm_msgs::FSMService::Response &response);

private:

  ros::ServiceServer fsm_service_;
  ros::Timer timer_;


   PathMatching path_matching_;
};

}
