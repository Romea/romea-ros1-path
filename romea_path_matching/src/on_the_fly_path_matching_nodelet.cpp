#include "on_the_fly_path_matching_nodelet.hpp"
#include <romea_common_utils/params/ros_param.hpp>

namespace romea{

//-----------------------------------------------------------------------------
OnTheFlyPathMatchingNodelet::OnTheFlyPathMatchingNodelet()
{

}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatchingNodelet::onConfigure()
{
  auto & nh = getNodeHandle();
  auto & private_nh = getPrivateNodeHandle();

  path_matching_.init(nh,private_nh);
  timer_= nh.createTimer(ros::Rate(1),&OnTheFlyPathMatching::timerCallback,(OnTheFlyPathMatching*)&path_matching_,false,false);
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatchingNodelet::onActivate()
{
  timer_.start();
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatchingNodelet::onDeactivate()
{
  timer_.stop();
}


}

PLUGINLIB_EXPORT_CLASS(romea::OnTheFlyPathMatchingNodelet, nodelet::Nodelet);
