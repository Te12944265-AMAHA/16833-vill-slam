cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")

execute_process(COMMAND lsb_release -cs
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
)

if(${LSB_RELEASE_ID_SHORT} MATCHES "bionic")
  set(CMAKE_CXX_FLAGS "-std=c++11")
  message("-- Systemm version: ${LSB_RELEASE_ID_SHORT}, set to C++11")
elseif(${LSB_RELEASE_ID_SHORT} MATCHES "focal")
  set(CMAKE_CXX_FLAGS "-std=c++14")
  message("-- Systemm version: ${LSB_RELEASE_ID_SHORT}, set to C++14")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
)

target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})
