//ros
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

//romea
#include "wgs84_path_matching.hpp"
#include <romea_fsm_msgs/FSMService.h>

namespace romea {


class WGS84PathMatchingNodelet : public nodelet::Nodelet
{
public:

  WGS84PathMatchingNodelet();

  virtual ~WGS84PathMatchingNodelet();

  virtual void onInit()override;

protected :

  bool serviceCallback_(romea_fsm_msgs::FSMService::Request  &request,
                        romea_fsm_msgs::FSMService::Response &response);

private:

  ros::ServiceServer fsm_service_;
  ros::Timer timer_;


   WGS84PathMatching path_matching_;
};

}
