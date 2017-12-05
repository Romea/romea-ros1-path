cmake_minimum_required(VERSION 2.8.3)
project(romea_path_msgs)

set(${PROJECT_NAME}_CATKIN_DEPS std_msgs romea_localisation_msgs message_generation)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

add_message_files(
  DIRECTORY msg
  FILES
  ENUPathPosture2D.msg
  ENUPathFrenetPose2D.msg
  ENUPathMatchedPoint2D.msg
  ENUPathMatchedPoint2DStamped.msg
  ENUPathMatchedPointAndBodyTwist2D.msg
  ENUPathMatchedPointAndBodyTwist2DStamped.msg

  )

generate_messages(DEPENDENCIES std_msgs romea_localisation_msgs)

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})
