cmake_minimum_required(VERSION 2.8.3)
project(camera_model)

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
set (CMAKE_EXE_LINKER_FLAGS "-lboost_filesystem")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    )

if(${LSB_RELEASE_ID_SHORT} MATCHES "bionic")
  find_package(Boost 1.65.1 EXACT REQUIRED COMPONENTS filesystem program_options system)
elseif(${LSB_RELEASE_ID_SHORT} MATCHES "focal")
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
endif()
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
# find_package(Ceres REQUIRED)
#set(Ceres_DIR /usr/local/ceres-1.4)
find_package(Ceres 1.14.0 EXACT REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES camera_model
    CATKIN_DEPENDS roscpp std_msgs
#    DEPENDS system_lib
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

include_directories("include")


add_executable(Calibration 
    src/intrinsic_calib.cc
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

add_library(camera_model
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(Calibration ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
