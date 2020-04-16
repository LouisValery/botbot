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

if(WIN32)
    SET(SL_IOT_BIN_DIR "$ENV{ZED_SDK_ROOT_DIR}/bin")
    SET(SL_IOT_LIB_DIR "$ENV{ZED_SDK_ROOT_DIR}/lib")
    SET(SL_IOT_INCLUDE_DIR "$ENV{ZED_SDK_ROOT_DIR}/include")
else(WIN32)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
    add_definitions(-Wno-write-strings)
    SET(SL_IOT_LIB_DIR /usr/local/sl_iot/lib/)
    SET(SL_IOT_INCLUDE_DIR /usr/local/sl_iot/include/)
    SET(UTILS_LIB pthread util curl dl)
endif(WIN32)

## Declare a catkin package
catkin_package()
# INCLUDES

# Specify locations of header files.
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
        ${SL_IOT_INCLUDE_DIR}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${SL_IOT_LIB_DIR})

include_directories(include ${catkin_INCLUDE_DIRS})

## Build 

add_executable(people_tracking src/people_tracking.cpp)
target_link_libraries(people_tracking ${catkin_LIBRARIES})

add_dependencies(people_tracking ${catkin_EXPORTED_TARGETS})

SET(ZED_LIBS_EXT sl_iot ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
target_link_libraries(people_tracking
                        ${SPECIAL_OS_LIBS}
                        ${ZED_LIBS_EXT}
                        ${OPENSSL_LIBRARIES} ${UTILS_LIB})


