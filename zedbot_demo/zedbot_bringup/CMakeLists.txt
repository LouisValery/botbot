

cmake_minimum_required(VERSION 2.8.3)
project(zedbot_bringup)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)



###################################
## catkin specific configuration ##
###################################

catkin_package()

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

###############################################################################

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(zed_orientation_publisher src/zed_orientation_publisher.cpp)
target_link_libraries(zed_orientation_publisher ${catkin_LIBRARIES})


