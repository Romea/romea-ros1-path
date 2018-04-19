#ifndef romea_WGS84Path2D_hpp
#define romea_WGS84Path2D_hpp

//romea
#include "Path2D.hpp"
#include <geodesy/ENUConverter.hpp>

namespace romea {

struct WGS84Path2D
{

  WGS84Path2D();

  WGS84Path2D(const GeodeticCoordinates & anchor);

  WGS84Path2D(const GeodeticCoordinates & anchor,
              const VectorOfEigenVector2d & enuPoints);

  void setAnchor(const WGS84Coordinates & anchor);

  Path2D enuPath;

  ENUConverter enuCoordinateSystems;

};

}


#endif
