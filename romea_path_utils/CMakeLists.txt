cmake_minimum_required(VERSION 3.0.2)
project(romea_path_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#######################
## Find dependencies ##
#######################
set(${PROJECT_NAME}_CATKIN_DEPS romea_common_utils romea_path_msgs)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(romea_core_common REQUIRED)
find_package(romea_core_path REQUIRED)
find_package(GSL REQUIRED)
find_package(BLAS REQUIRED)


find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include ${romea_path_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

add_library(romea_path_utils
  src/path_frenet_pose2d_conversions.cpp
  src/path_matching_info_conversions.cpp
  src/path_matching_point2d_conversions.cpp
  src/path_posture2d_conversions.cpp
  src/path_builder.cpp
)

target_include_directories(romea_path_utils PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${romea_core_path_INCLUDE_DIRS}
)

target_link_libraries(romea_path_utils PUBLIC
  ${catkin_LIBRARIES}
  romea_core_common::romea_core_common
  romea_core_path::romea_core_path
)

install(TARGETS romea_path_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/romea_path_utils/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)
if(CATKIN_ENABLE_TESTING)

    catkin_add_gtest(${PROJECT_NAME}_test_ros_conversion test/test_ros_conversion.cpp )
    target_link_libraries(${PROJECT_NAME}_test_ros_conversion ${PROJECT_NAME})

endif(CATKIN_ENABLE_TESTING)

