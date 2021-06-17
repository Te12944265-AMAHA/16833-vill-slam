//
// Created by dcheng on 4/11/20.
//

#include <iostream>
#include <ros/ros.h>
#include <blaser_ros/laser_detector.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_detector");

  std::string config_fn(argv[1]);
  std::string config_ns;

  if (argc == 3)
    config_ns = argv[2];
  if (argc == 2)
    config_ns = "";

  LaserDetector ld(config_fn, "~", config_ns);

  ros::spin();

  return 0;
}