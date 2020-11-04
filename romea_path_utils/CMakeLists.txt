cmake_minimum_required(VERSION 2.8.3)
project(romea_path_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17 -O3")

#######################
## Find dependencies ##
#######################
set(${PROJECT_NAME}_CATKIN_DEPS romea_path_msgs romea_localisation_utils)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(romea_common REQUIRED)
find_package(romea_localisation REQUIRED)
find_package(romea_path REQUIRED)
find_package(GSL REQUIRED)
find_package(BLAS REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include ${romea_path_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

###########
## Build ##
###########

include_directories(
    include
    SYSTEM ${catkin_INCLUDE_DIRS}
    SYSTEM ${romea_path_INCLUDE_DIRS}
    )

add_library(romea_path_utils
    src/PathFrenetPose2DConversions.cpp
    src/PathMatchingInfoConversions.cpp
    src/PathMatchingPoint2DConversions.cpp
    src/PathPosture2DConversions.cpp
    )

target_link_libraries(romea_path_utils
  ${catkin_LIBRARIES}
  romea_path::romea_path
  )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS trackbod_driver trackbod_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)
if(CATKIN_ENABLE_TESTING)

    catkin_add_gtest(${PROJECT_NAME}_test_ros_conversion test/test_ros_conversion.cpp )
    target_link_libraries(${PROJECT_NAME}_test_ros_conversion ${PROJECT_NAME})

endif(CATKIN_ENABLE_TESTING)

