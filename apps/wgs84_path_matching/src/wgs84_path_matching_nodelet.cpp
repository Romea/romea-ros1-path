#include "wgs84_path_matching_nodelet.hpp"


namespace romea{

//-----------------------------------------------------------------------------
WGS84PathMatchingNodelet::WGS84PathMatchingNodelet()
{

}

//-----------------------------------------------------------------------------
WGS84PathMatchingNodelet::~WGS84PathMatchingNodelet()
{

}


//-----------------------------------------------------------------------------
void WGS84PathMatchingNodelet::onInit()
{

  auto & nh = getNodeHandle();
  auto & private_nh = getPrivateNodeHandle();

  if(path_matching_.init(nh,private_nh))
  {
    bool autostart;
    private_nh.param("autostart",autostart,true);
    timer_= nh.createTimer(ros::Rate(1),&WGS84PathMatching::publishTf,(WGS84PathMatching*)&path_matching_,false,autostart);
    fsm_service_ = private_nh.advertiseService("fsm_service",&WGS84PathMatchingNodelet::serviceCallback_,this);
  }
}


//-----------------------------------------------------------------------------
bool WGS84PathMatchingNodelet::serviceCallback_(romea_fsm_srvs::FSMService::Request  &request,
                                                romea_fsm_srvs::FSMService::Response &response)
{
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
    response.success=path_matching_.loadPath(request.command_arguments);
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

PLUGINLIB_EXPORT_CLASS(romea::WGS84PathMatchingNodelet, nodelet::Nodelet);
