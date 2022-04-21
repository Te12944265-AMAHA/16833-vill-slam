# Install script for directory: /home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/pipe_blaser_ros

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pipe_blaser_ros" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/include/pipe_blaser_ros/LaserRingDetectorConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/pipe_blaser_ros" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/lib/python2.7/dist-packages/pipe_blaser_ros/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/lib/python2.7/dist-packages/pipe_blaser_ros/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/pipe_blaser_ros" TYPE DIRECTORY FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/devel/lib/python2.7/dist-packages/pipe_blaser_ros/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/pipe_blaser_ros/catkin_generated/installspace/pipe_blaser_ros.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pipe_blaser_ros/cmake" TYPE FILE FILES
    "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/pipe_blaser_ros/catkin_generated/installspace/pipe_blaser_rosConfig.cmake"
    "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/build/pipe_blaser_ros/catkin_generated/installspace/pipe_blaser_rosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pipe_blaser_ros" TYPE FILE FILES "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/pipe_blaser_ros/package.xml")
endif()

