cmake_minimum_required(VERSION 3.0.2)
project(romea_path_matching)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

#######################
## Find dependencies ##
#######################
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    roslib
    nodelet
    romea_path_utils
    romea_lifecycle
    tf2_eigen #hack to include eigen
    )

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(romea_common REQUIRED)
find_package(romea_path REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  DEPENDS romea_common romea_path
  )


###########
## build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

add_library(path_matching_nodelet
    src/path_file.cpp
    src/path_matching.cpp
    src/path_matching_base.cpp
    src/path_matching_diagnostic.cpp
    src/path_matching_diagnostic_base.cpp
    src/path_matching_display.cpp
    src/path_matching_display_base.cpp
    src/path_matching_tf.cpp
    src/path_matching_nodelet.cpp
    )

add_library(on_the_fly_path_matching_nodelet
    src/path_matching_base.cpp
    src/path_matching_diagnostic_base.cpp
    src/path_matching_display_base.cpp
    src/on_the_fly_path_matching.cpp
    src/on_the_fly_path_matching_diagnostic.cpp
    src/on_the_fly_path_matching_display.cpp
    src/on_the_fly_path_matching_nodelet.cpp
    )

add_executable(path_matching_node src/path_matching_node.cpp)
add_executable(on_the_fly_path_matching_node src/on_the_fly_path_matching_node.cpp)

target_link_libraries(path_matching_node ${catkin_LIBRARIES})
target_link_libraries(path_matching_nodelet ${catkin_LIBRARIES})

target_link_libraries(on_the_fly_path_matching_node ${catkin_LIBRARIES})
target_link_libraries(on_the_fly_path_matching_nodelet ${catkin_LIBRARIES})

