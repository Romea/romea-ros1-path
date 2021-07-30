#ifndef _romea_PathMatchingInfoConversions_hpp_
#define _romea_PathMatchingInfoConversions_hpp

//romea
#include <romea_path/PathMatchedPoint2D.hpp>
#include <romea_path_msgs/PathMatchingInfo2D.h>
#include <romea_localisation/Twist2D.hpp>
#include <romea_common/time/Time.hpp>

namespace romea {


romea_path_msgs::PathMatchingInfo2D toRosMsg(const Duration & duration,
                                             const PathMatchedPoint2D matched_point,
                                             const double &path_length,
                                             const double & future_curvature,
                                             const Twist2D & twist);

romea_path_msgs::PathMatchingInfo2D toRosMsg(const ros::Time & stamp,
                                             const PathMatchedPoint2D matched_point,
                                             const double & path_length,
                                             const double & future_curvature,
                                             const Twist2D & twist);

}

#endif
