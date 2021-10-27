cmake_minimum_required(VERSION 3.0.2)
project(romea_path_msgs)

set(${PROJECT_NAME}_CATKIN_DEPS std_msgs romea_common_msgs message_generation)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

add_message_files(
  DIRECTORY msg
  FILES
  PathPosture2D.msg
  PathFrenetPose2D.msg
  PathMatchedPoint2D.msg
  PathMatchingInfo2D.msg
  )

generate_messages(DEPENDENCIES std_msgs romea_common_msgs)

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})
