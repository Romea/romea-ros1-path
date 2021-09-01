#ifndef PathMatchingDysplay_HPP
#define PathMatchingDisplay_HPP


//romea
#include "path_matching_display_base.hpp"

namespace romea {

class PathMatchingDisplay : public PathMatchingDisplayBase
{

public :

using WayPoints = std::vector<std::vector<PathWayPoint2D>>;

public :

  PathMatchingDisplay();

  virtual ~PathMatchingDisplay()=default;

  void loadWayPoints(const WayPoints & path_way_points);

protected:

  void addWayPoint_(const PathWayPoint2D & point);

};

}
#endif
