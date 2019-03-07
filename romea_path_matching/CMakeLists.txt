cmake_minimum_required(VERSION 2.8.3)
project(romea_path_matching)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11")

#######################
## Find dependencies ##
#######################
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

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})


###########
## build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

add_library(path_matching_nodelet
    src/path_matching.cpp
    src/path_matching_diagnostic.cpp
    src/path_matching_nodelet.cpp)

add_executable(path_matching_node
    src/path_matching_node.cpp)

target_link_libraries(path_matching_node ${catkin_LIBRARIES})
target_link_libraries(path_matching_nodelet ${catkin_LIBRARIES})

