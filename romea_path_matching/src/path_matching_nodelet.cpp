#include "path_matching_nodelet.hpp"
#include <filesystem/FileSystem.hpp>

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

  std::cout << " PathMatchingNodelet::onInit() "<< std::endl;
  if(path_matching_.init(nh,private_nh))
  {
    bool autostart;
    private_nh.param("autostart",autostart,true);

    bool revert;
    private_nh.param("revert",revert,false);

    std::string path_filename;
    if(private_nh.getParam("path",path_filename))
    {
      std::cout << " path_filename "<< path_filename << std::endl;
      if(!path_matching_.loadPath(path_filename,revert))
      {
        ROS_ERROR("Unable to load path %s",path_filename.c_str());
        autostart=false;
      }
    }
    else
    {
      if(autostart)
      {
        ROS_ERROR("Cannot load path filename from param server");
      }
    }

    timer_= nh.createTimer(ros::Rate(1),&PathMatching::publishTf,(PathMatching*)&path_matching_,false,autostart);
    fsm_service_ = private_nh.advertiseService("fsm_service",&PathMatchingNodelet::serviceCallback_,this);

  }
  else
    ROS_ERROR("Cannot init path matching");
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
