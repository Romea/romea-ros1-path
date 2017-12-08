cmake_minimum_required(VERSION 2.8.3)
project(romea_path_msgs)

set(${PROJECT_NAME}_CATKIN_DEPS std_msgs romea_localisation_msgs message_generation)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

add_message_files(
  DIRECTORY msg
  FILES
  PathPosture2D.msg
  PathFrenetPose2D.msg
  PathMatchedPoint2D.msg
  PathMatchedPoint2DStamped.msg
  PathMatchedPointAndTwist2D.msg
  PathMatchedPointAndTwist2DStamped.msg

  )

generate_messages(DEPENDENCIES std_msgs romea_localisation_msgs)

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})
