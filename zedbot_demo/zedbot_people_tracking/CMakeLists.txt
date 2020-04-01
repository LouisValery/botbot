cmake_minimum_required(VERSION 2.8.3)
project(zedbot_people_tracking)


# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  zed_wrapper
  tf
 )

generate_messages(DEPENDENCIES
    zed_wrapper
)

## Declare a catkin package
catkin_package()
# INCLUDES

# Specify locations of header files.
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})


include_directories(include ${catkin_INCLUDE_DIRS})

## Build 

add_executable(people_tracking src/people_tracking.cpp)
target_link_libraries(people_tracking ${catkin_LIBRARIES})

add_dependencies(zed_obj_det_sub ${catkin_EXPORTED_TARGETS})
