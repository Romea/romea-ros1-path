cmake_minimum_required(VERSION 2.8.3)
project(romea_wgs84_path_matching)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11")

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    roslib
    nodelet
    tf
    romea
    romea_path
    romea_localisation
    romea_fsm_msgs
    )

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

include_directories(${catkin_INCLUDE_DIRS} /usr/include/eigen3)

add_library(wgs84_path_matching_nodelet
    src/wgs84_path_matching.cpp
    src/wgs84_path_matching_diagnostic.cpp
    src/wgs84_path_matching_nodelet.cpp)

add_executable(wgs84_path_matching_node
    src/wgs84_path_matching_node.cpp
)

target_link_libraries(wgs84_path_matching_node ${catkin_LIBRARIES})
target_link_libraries(wgs84_path_matching_nodelet ${catkin_LIBRARIES})

