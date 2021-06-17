//
// Created by dcheng on 3/23/20.
//

#include <iostream>
#include <blaser_ros/laser_calib.h>
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


using std::cout;
using std::endl;


int main(int argc, char **argv)
{
  /*
  std::string image_dir(argv[1]);
  std::string target_config_fn(argv[2]), env_config_fn(argv[3]),
      cam_config_fn(argv[4]);
  std::string cam_model(argv[5]);
  bool f_calib_laser_ori = (std::string(argv[6]) == "1");
   */

  std::string image_dir;
  std::string config_file;
  std::string cam_model; // pinhole or camodocal
  bool calib_laser_origin;

  // handle program options
  namespace po = boost::program_options;
  po::options_description desc("Laser calib allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("config,c", po::value<std::string>(&config_file), "config file")
      ("input,i", po::value<std::string>(&image_dir), "image input directory")
      ("camera-model",
          po::value<std::string>(&cam_model)->default_value("camodocal"),
          "pinhole | camodocal (preferred)")
      ("calib-origin",
          po::bool_switch(&calib_laser_origin)->default_value(false),
          "calibrate laser origin, default to false")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << endl;
    return 1;
  }

  if (!boost::filesystem::exists(image_dir) && !boost::filesystem::is_directory(image_dir))
  {
    std::cerr << "# ERROR: Cannot find image input directory:" << endl
              << image_dir << endl;
    return 1;
  }

  auto laser_calib = createLaserCalib(cam_model, image_dir, target_config_fn,
                                      env_config_fn, cam_config_fn,
                                      f_calib_laser_ori);

  laser_calib->examineByImage();

  Eigen::Vector4d plane_param;
  laser_calib->solveLaserPlane(plane_param);

  Eigen::Vector3d laser_ori, tcl;
  Eigen::Matrix3d Rcl;
  Eigen::Vector2d fan_lb, fan_rb;
  laser_calib->solveLaserParams(plane_param, laser_ori, Rcl, tcl, fan_lb,
                                fan_rb);
}