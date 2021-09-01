#ifndef PathMatchingTf_HPP
#define PathMatchingTf_HPP

//ros
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


namespace romea {

class PathMatchingTf
{

public :

  PathMatchingTf();

  void init(ros::NodeHandle & private_nh);

  void setWorldToPathTransformation(const Eigen::Affine3d & tf);

  const Eigen::Affine3d &getMapToPathTransformation() const;

  bool evaluteMapToPathTransformation(const ros::Time & stamp,
                                      const std::string & map_frame_id);

  void publish(const ros::Time &stamp);

  void reset();


private:

  Eigen::Affine3d map_to_path_;
  Eigen::Affine3d world_to_path_;
  Eigen::Affine3d world_to_map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped tf_world_to_path_msg_;


  bool is_tf_loaded;
  bool publish_;

};

}
#endif
