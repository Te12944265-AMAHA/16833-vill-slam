//
// Created by dcheng on 3/22/20.
//

#include <blaser_ros/laser_stripe_detector.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  std::string env_config_fn(argv[2]);
  cv::Mat im = cv::imread(argv[1]);
  cv::Mat im_undistort;
  cv::Mat K = (cv::Mat_<double>(3,3) <<
      340.810907, 0.000000, 328.921902,
      0.000000, 341.291995, 224.532553,
      0.000000, 0.000000, 1.000000);
  cv::Mat D = (cv::Mat_<double>(1,5) <<
  -0.263077, 0.046754, -0.001104, -0.000508, 0.000000);

  cv::undistort(im, im_undistort, K, D);

  LaserStripeDetector lsd(env_config_fn, "1");
  std::vector<cv::Point2f> laser_pts;
  lsd.detectLaserStripe(im_undistort, laser_pts);

  for (const auto laser_pt : laser_pts)
    cv::circle(im_undistort, laser_pt, 0, cv::Scalar(0, 255, 0), 2);

  cv::imshow("laser undistort", im_undistort);
  cv::waitKey(1000000);

  return 0;
}