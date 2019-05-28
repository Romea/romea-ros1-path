#include "path_matching_nodelet.hpp"
#include <filesystem/FileSystem.hpp>
#include <ros/Factory.hpp>

namespace romea{

//-----------------------------------------------------------------------------
PathMatchingNodelet::PathMatchingNodelet()
{

}

//-----------------------------------------------------------------------------
void PathMatchingNodelet::onInit()
{
  auto & nh = getNodeHandle();
  auto & private_nh = getPrivateNodeHandle();

  try
  {
    std::string path_filename = loadParam<std::string>(private_nh,"path");
    bool autostart= private_nh.param("autostart",true);
    bool revert=private_nh.param("revert",false);

    path_matching_.init(nh,private_nh);
    path_matching_.loadPath(path_filename,revert);

    timer_= nh.createTimer(ros::Rate(1),&PathMatching::publishTf,(PathMatching*)&path_matching_,false,autostart);
    fsm_service_ = private_nh.advertiseService("fsm_service",&PathMatchingNodelet::serviceCallback_,this);

  }
  catch(std::exception & e)
  {
    ROS_ERROR_STREAM("PathMatchingNodelet : "<<e.what());
  }
}


//-----------------------------------------------------------------------------
bool PathMatchingNodelet::serviceCallback_(romea_fsm_msgs::FSMService::Request  &request,
                                           romea_fsm_msgs::FSMService::Response &response)
{

  std::cout << " service " << request.command << std::endl;
  if(request.command.compare("start")==0)
  {
    timer_.start();
    response.success=true;
  }
  else if(request.command.compare("stop")==0)
  {
    timer_.stop();
    response.success=true;
  }
  else if(request.command.compare("loadPath")==0)
  {
    bool running = timer_.hasPending();
    if(running)
    {
      timer_.stop();
    }

    path_matching_.reset();
    response.success=path_matching_.loadPath(request.command_arguments,false);
    if(!response.success)
    {
      response.message= "file "+ request.command_arguments+ " cannot be loaded";
      return false;
    }

    if(running)
    {
      timer_.start();
    }
  }
  else
  {
    response.success=false;
    return false;
  }

  return true;
}

}

PLUGINLIB_EXPORT_CLASS(romea::PathMatchingNodelet, nodelet::Nodelet);
