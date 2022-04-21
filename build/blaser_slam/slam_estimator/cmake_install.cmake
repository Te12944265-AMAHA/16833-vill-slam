# Install script for directory: /home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/blaser_slam/slam_estimator

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slam_estimator/msg" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/blaser_slam/slam_estimator/msg/ResidualCostMsg.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slam_estimator/cmake" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/blaser_slam/slam_estimator/catkin_generated/installspace/slam_estimator-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/include/slam_estimator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/share/roseus/ros/slam_estimator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/share/common-lisp/ros/slam_estimator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/share/gennodejs/ros/slam_estimator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/lib/python2.7/dist-packages/slam_estimator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/lib/python2.7/dist-packages/slam_estimator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/slam_estimator" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/include/slam_estimator/BlaserSLAMConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/slam_estimator" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/lib/python2.7/dist-packages/slam_estimator/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/lib/python2.7/dist-packages/slam_estimator/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/slam_estimator" TYPE DIRECTORY FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/lib/python2.7/dist-packages/slam_estimator/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/blaser_slam/slam_estimator/catkin_generated/installspace/slam_estimator.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slam_estimator/cmake" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/blaser_slam/slam_estimator/catkin_generated/installspace/slam_estimator-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slam_estimator/cmake" TYPE FILE FILES
    "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/blaser_slam/slam_estimator/catkin_generated/installspace/slam_estimatorConfig.cmake"
    "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/blaser_slam/slam_estimator/catkin_generated/installspace/slam_estimatorConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/slam_estimator" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/blaser_slam/slam_estimator/package.xml")
endif()

