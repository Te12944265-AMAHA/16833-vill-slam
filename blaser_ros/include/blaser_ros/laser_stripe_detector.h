//
// Created by dcheng on 3/21/20.
//

#ifndef VIO_BLASER_LASER_STRIPE_DETECTOR_H
#define VIO_BLASER_LASER_STRIPE_DETECTOR_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

/*
class LaserExtractParam
{
public:
  LaserExtractParam();

  LaserExtractParam(int brightness_thresh,
                    int hue_thresh_1,
                    int hue_thresh_2);

  void setParams(int brightness_thresh,
                 int hue_thresh_1,
                 int hue_thresh_2);

  void genHSVScalars(cv::Scalar& red_mask_1_l,
                     cv::Scalar& red_mask_1_h,
                     cv::Scalar& red_mask_2_l,
                     cv::Scalar& red_mask_2_h) const;

private:
  int brightness_thresh_;
  int hue_thresh_1_;
  int hue_thresh_2_;
};
 */

class LaserStripeDetector
{
public:
  /**
   * Constructor function.
   * @param env_config_fn yaml file describing laser detection parameters
   */
  explicit LaserStripeDetector(std::string &env_config_fn, std::string ns);

  /**
   * Constructor function that also enables camera undistortion. Need to provide
   * camera intrinsics
   * @param env_config_fn yaml file describing laser detection parameters
   * @param cam_config_fn yaml file describing camera parameters
   */
  explicit LaserStripeDetector(std::string &env_config_fn,
                               std::string &cam_config_fn,
                               std::string ns);

  /**
   * Function to detect laser points in the given image
   * @param im input image
   * @param laser_pts output laser pixels
   * @return false if no laser pixels are found
   */
  bool detectLaserStripe(cv::Mat& im, std::vector<cv::Point2f> &laser_pts);

  /**
   * Return the undistorted laser pixels. Make sure that the intrinsics are
   * provided before hand, and that the image is not undistorted.
   * @param im undistorted image
   * @param laser_pts output: undistorted laser pixels
   * @return false if no laser pixels are detected
   */
  bool detectLaserStripeUndistort(cv::Mat& im,
      std::vector<cv::Point2f> &laser_pts);

  /**
   * Set the laser extraction param, resulting HSV thresholds:
   * red_mask_1_h = [180, 255, 255]
   * red_mask_1_l = [hue_thresh_1, 170, brightness_thresh]
   * red_mask_2_h = [hue_thresh_2, 255, 255]
   * red_mask_2_l = [0, 170, brightness_thresh]
   * @param brightness_thresh
   * @param hue_thresh_1
   * @param hue_thresh_2
   * @return true if all params are within range
   */
  bool setLaserExtractParams(int brightness_thresh,
                             int hue_thresh_1,
                             int hue_thresh_2);

private:
  /**
   * Find the Center of Mass of a high-valued window on each column of the
   * given image. If there does not exist any pixel with a high value (>= 10)
   * on a column, then this column is discarded.
   * @param im
   * @param CoM_pts
   * @return false if no points are selected.
   */
  bool findCoMColumn(Eigen::MatrixXd &im, std::vector<cv::Point2f> &CoM_pts);

  // functions to load config yaml files
  void loadEnvConfig(std::string& env_config_fn);
  void loadCamConfig(std::string& cam_config_fn);

  void rejectOutliers(const std::vector<cv::Point2f> &pts_in,
      std::vector<cv::Point2f> &pts_out);

  /**
   * Initialize algorithm parameters that are not pass in through config files
   */
  void initParams();

  // two red threshold, lower and higher bounds
  cv::Scalar red_mask_1_l_, red_mask_1_h_, red_mask_2_l_, red_mask_2_h_;

  double v_thresh_, v_thresh_ratio_;

  // region of interest where we try to find laser stripe
  cv::Rect laser_ROI_;

  // camera intrinsics
  const bool f_intrisics_;
  cv::Mat K_;
  cv::Mat D_;

  std::string ns_;//namespace of config

  static const int MIN_LASER_PTS = 20;

  // algorithm parameters
  cv::Mat mask_dilate_kernel_, mask_close_kernel_;
};

#endif //VIO_BLASER_LASER_STRIPE_DETECTOR_H
