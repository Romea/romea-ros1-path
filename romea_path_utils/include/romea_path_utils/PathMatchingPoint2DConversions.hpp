#ifndef _romea_PathMatchingPoint2DConversions_hpp_
#define _romea_PathMatchingPoint2DConversions_hpp

//romea
#include <romea_path/PathMatchedPoint2D.hpp>
#include <romea_path_msgs/PathMatchedPoint2D.h>

namespace romea {


void toRosMsg(const PathMatchedPoint2D & romea_matched_point2d,
              romea_path_msgs::PathMatchedPoint2D & ros_path_matched_point2d_msg);

void toRomea(const romea_path_msgs::PathMatchedPoint2D & matched_point_msg,
             PathMatchedPoint2D & romea_matched_point);

PathMatchedPoint2D toRomea(const romea_path_msgs::PathMatchedPoint2D &matched_point_msg);

}

#endif
