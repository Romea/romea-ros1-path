#ifndef romea_WGS84KalmanLocalisationFilter_hpp
#define romea_WGS84KalmanLocalisationFilter_hpp

//romea
#include "ENUPath2D.hpp"
#include <geodesy/ENUConverter.hpp>

namespace romea {

struct WGS84Path2D
{

  WGS84Path2D();

  WGS84Path2D(const GeodeticCoordinates & anchor);

  WGS84Path2D(const GeodeticCoordinates & anchor,
              const VectorOfEigenVector<Eigen::Vector2d> & enuPoints);

  void setAnchor(const WGS84Coordinates & anchor);

  ENUPath2D enuPath;

  ENUConverter enuCoordinateSystems;

};

}


#endif
