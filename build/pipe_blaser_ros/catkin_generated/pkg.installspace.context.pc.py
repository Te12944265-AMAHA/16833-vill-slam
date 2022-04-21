# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "camera_model;cv_bridge;dynamic_reconfigure;geometry_msgs;pcl_conversions;pcl_ros;roscpp;sensor_msgs;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lpipe_blaser_ros".split(';') if "-lpipe_blaser_ros" != "" else []
PROJECT_NAME = "pipe_blaser_ros"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
