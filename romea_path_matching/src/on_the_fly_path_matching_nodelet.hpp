#ifndef OnTheFlyPathMatchingNodelet_HPP
#define OnTheFlyPathMatchingNodelet_HPP

//ros
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

//romea
#include "on_the_fly_path_matching.hpp"
#include <romea_lifecycle/LifecycleNodelet.hpp>

namespace romea {

class OnTheFlyPathMatchingNodelet : public LifecycleNodelet
{
public:

  OnTheFlyPathMatchingNodelet();

  virtual ~OnTheFlyPathMatchingNodelet()=default;

  virtual void onConfigure()override;

  virtual void onActivate()override;

  virtual void onDeactivate()override;

private :

  ros::Timer timer_;
  OnTheFlyPathMatching path_matching_;
};

}

#endif
