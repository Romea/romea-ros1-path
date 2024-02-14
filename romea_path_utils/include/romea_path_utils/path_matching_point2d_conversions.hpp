#ifndef _romea_PathMatchingPoint2DConversions_hpp_
#define _romea_PathMatchingPoint2DConversions_hpp

//romea
#include <romea_core_path/PathMatchedPoint2D.hpp>
#include <romea_path_msgs/PathMatchedPoint2D.h>

namespace romea {


void to_ros_msg(const core::PathMatchedPoint2D & romea_matched_point2d,
              romea_path_msgs::PathMatchedPoint2D & ros_path_matched_point2d_msg);

void to_romea(const romea_path_msgs::PathMatchedPoint2D & matched_point_msg,
             core::PathMatchedPoint2D & romea_matched_point);

core::PathMatchedPoint2D to_romea(const romea_path_msgs::PathMatchedPoint2D &matched_point_msg);

}

#endif
