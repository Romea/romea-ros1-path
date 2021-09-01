//romea
#include "path_matching_tf.hpp"
#include <romea_common_utils/conversions/transform_conversions.hpp>
#include <romea_common_utils/params/ros_param.hpp>

//ros
#include <eigen_conversions/eigen_msg.h>


namespace romea {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatchingTf::PathMatchingTf():
  map_to_path_(),
  world_to_path_(),
  world_to_map_(),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  tf_broadcaster_(),
  tf_world_to_path_msg_(),
  is_tf_loaded(false)
{  
}


//-----------------------------------------------------------------------------
void PathMatchingTf::publish(const ros::Time &stamp)
{
    tf_world_to_path_msg_.header.stamp=stamp;
    toRosTransformMsg(world_to_path_,tf_world_to_path_msg_.transform);
    tf_broadcaster_.sendTransform(tf_world_to_path_msg_);
}

//-----------------------------------------------------------------------------
void PathMatchingTf::init(ros::NodeHandle & private_nh)
{

  std::string path_frame_id= load_param<std::string>(private_nh,"path_frame_id");
  tf_world_to_path_msg_.header.frame_id = "world";
  tf_world_to_path_msg_.child_frame_id =path_frame_id;
}


//-----------------------------------------------------------------------------
void PathMatchingTf::setWorldToPathTransformation(const Eigen::Affine3d & tf)
{
  world_to_path_=tf;
  is_tf_loaded=true;
}

//-----------------------------------------------------------------------------
const Eigen::Affine3d & PathMatchingTf::getMapToPathTransformation() const
{
  return map_to_path_;
}

//-----------------------------------------------------------------------------
bool PathMatchingTf::evaluteMapToPathTransformation(const ros::Time &stamp,
                                                    const std::string & map_frame_id)
{

  if(!is_tf_loaded)
  {
    return false;
  }

//  tf_world_to_path_msg_.header.stamp=stamp;

  try
  {
#warning anti date can cause trouble if map reference frame change
    geometry_msgs::TransformStamped transformStamped;
    transformStamped=tf_buffer_.lookupTransform("world",map_frame_id,stamp - ros::Duration(0.2));
    world_to_map_ = tf2::transformToEigen(transformStamped);
    map_to_path_ = tf2::transformToEigen(transformStamped).inverse()*world_to_map_;
    return true;
  }
  catch (tf2::TransformException ex)
  {
    std::cout << " catch " << std::endl;
    std::cout <<  ex.what() << std::endl;
    return false;
  }
}


//-----------------------------------------------------------------------------
void PathMatchingTf::reset()
{
  is_tf_loaded=false;
}


}
