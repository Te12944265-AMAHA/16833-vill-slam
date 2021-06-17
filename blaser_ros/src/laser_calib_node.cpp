//
// Created by dcheng on 3/23/20.
//

#include <iostream>
#include <blaser_ros/laser_calib.h>
#include <Eigen/Dense>

using std::cout;
using std::endl;

int main(int argc, char **argv)
{
  std::string image_dir(argv[1]);
  std::string target_config_fn(argv[2]), env_config_fn(argv[3]),
      cam_config_fn(argv[4]);
  std::string cam_model(argv[5]);
  bool f_calib_laser_ori = (std::string(argv[6]) == "1");

  auto laser_calib = createLaserCalib(cam_model, image_dir, target_config_fn,
      env_config_fn, cam_config_fn, f_calib_laser_ori);

  laser_calib->examineByImage();

  Eigen::Vector4d plane_param;
  laser_calib->solveLaserPlane(plane_param);

  Eigen::Vector3d laser_ori, tcl;
  Eigen::Matrix3d Rcl;
  Eigen::Vector2d fan_lb, fan_rb;
  laser_calib->solveLaserParams(plane_param, laser_ori, Rcl, tcl, fan_lb, fan_rb);
}