#include "WGS84Path2D.hpp"

namespace romea {

//-----------------------------------------------------------------------------
WGS84Path2D::WGS84Path2D():
  enuPath(),
  enuCoordinateSystems()
{

}

//-----------------------------------------------------------------------------
WGS84Path2D::WGS84Path2D(const GeodeticCoordinates & anchor):
  enuPath(),
  enuCoordinateSystems(anchor)
{

}

//-----------------------------------------------------------------------------
WGS84Path2D::WGS84Path2D(const GeodeticCoordinates & anchor,
                         const VectorOfEigenVector2d & enuPoints):
  enuPath(enuPoints),
  enuCoordinateSystems(anchor)
{

}

}
