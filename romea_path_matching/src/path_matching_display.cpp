//romea
#include "path_matching_display.hpp"


namespace romea {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatchingDisplay::PathMatchingDisplay()
{
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void PathMatchingDisplay::loadWayPoints(const WayPoints &path_way_points)
{
  for(const auto & section_way_points : path_way_points )
  {
    for(const auto & way_point : section_way_points)
    {
      addWayPoint_(way_point);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void PathMatchingDisplay::addWayPoint_(const PathWayPoint2D & way_point)
{
  path3d_.emplace_back(way_point.position.x(),
                       way_point.position.y(),
                       0.1);
}



}
