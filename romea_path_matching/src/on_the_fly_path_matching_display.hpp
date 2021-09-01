#ifndef OnTheFlyPathMatchingDysplay_HPP
#define OnTheFlyPathMatchingDisplay_HPP


//romea
#include "path_matching_display_base.hpp"

namespace romea {

class OnTheFlyPathMatchingDisplay : public PathMatchingDisplayBase
{

public :

  OnTheFlyPathMatchingDisplay();

  virtual ~OnTheFlyPathMatchingDisplay()=default;

  void addPoint(const Eigen::Vector2d & point);


};

}
#endif
