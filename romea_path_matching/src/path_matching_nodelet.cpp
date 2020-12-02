#include "path_matching_nodelet.hpp"
#include <romea_common_utils/params/RosParam.hpp>

namespace romea{

//-----------------------------------------------------------------------------
PathMatchingNodelet::PathMatchingNodelet()
{

}

//-----------------------------------------------------------------------------
void PathMatchingNodelet::onConfigure()
{
  auto & nh = getNodeHandle();
  auto & private_nh = getPrivateNodeHandle();

  std::string path_filename = loadParam<std::string>(private_nh,"path");
  bool revert=private_nh.param("revert",false);

  path_matching_.init(nh,private_nh);
  path_matching_.loadPath(path_filename,revert);

  timer_= nh.createTimer(ros::Rate(1),&PathMatching::publishTf,(PathMatching*)&path_matching_,false,false);
  load_path_service_ = nh.advertiseService("load_path",&PathMatchingNodelet::loadPathCallback_,this);
}

//-----------------------------------------------------------------------------
void PathMatchingNodelet::onActivate()
{
  timer_.start();
}

//-----------------------------------------------------------------------------
void PathMatchingNodelet::onDeactivate()
{
  timer_.stop();
}

//-----------------------------------------------------------------------------
bool PathMatchingNodelet::loadPathCallback_(romea_path_msgs::LoadPath::Request & request,
                                            romea_path_msgs::LoadPath::Response &)
{
  bool running = timer_.hasPending();
  if(running)
  {
    timer_.stop();
  }

  try
  {
    path_matching_.loadPath(request.path_filename,request.revert);
  }
  catch(std::runtime_error & e)
  {
    return false;
  }

  if(running)
  {
    timer_.start();
  }

  return true;
}


}

PLUGINLIB_EXPORT_CLASS(romea::PathMatchingNodelet, nodelet::Nodelet);
