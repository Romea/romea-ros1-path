//romea
#include "on_the_fly_path_matching_display.hpp"


namespace romea {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
OnTheFlyPathMatchingDisplay::OnTheFlyPathMatchingDisplay()
{
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void OnTheFlyPathMatchingDisplay::addPoint(const Eigen::Vector2d & point)
{
  path3d_.emplace_back(point.x(),
                       point.y(),
                       0.1);

}

}
