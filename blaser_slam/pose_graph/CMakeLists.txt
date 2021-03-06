cmake_minimum_required(VERSION 2.8.3)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Debug")

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

#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    camera_model
    cv_bridge
    roslib
    pcl_ros
    pcl_conversions
    tf
    )

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

#find_package(Ceres REQUIRED)
find_package(Ceres 1.14.0 EXACT REQUIRED)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

catkin_package()

add_executable(pose_graph
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    src/map_point.cpp
    src/map.cpp
    src/nanoflann_map_point.cpp
    )
target_link_libraries(pose_graph ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES})

add_executable(nanoflann_map_test
    test/nanoflann_map_point_test.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    src/map_point.cpp
    src/map.cpp
    src/nanoflann_map_point.cpp
    )
target_link_libraries(nanoflann_map_test ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_ransac_test
    test/pcl_ransac_test.cpp)
target_link_libraries(pcl_ransac_test ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${PCL_LIBRARIES})

# message("catkin_lib  ${catkin_LIBRARIES}")
