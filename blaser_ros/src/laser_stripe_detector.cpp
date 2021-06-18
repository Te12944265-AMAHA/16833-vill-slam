//
// Created by dcheng on 3/22/20.
//

#include <blaser_ros/laser_stripe_detector.h>
#include <opencv2/core/eigen.hpp>

using std::cout;
using std::endl;
LaserStripeDetector::LaserStripeDetector(const std::string &config_fn)
: f_intrisics_(false)
, v_thresh_(0.0)
{
  loadEnvConfig(config_fn);
  initParams();
}

LaserStripeDetector::LaserStripeDetector(const std::string &env_config_fn,
                                         const std::string &cam_config_fn)
: f_intrisics_(true)
{
  loadEnvConfig(env_config_fn);
  loadCamConfig(cam_config_fn);
  initParams();
}

bool LaserStripeDetector::detectLaserStripe(cv::Mat &im,
                                            std::vector<cv::Point2f> &laser_pts) {
  cv::Mat im_blur, im_hsv, im_v;
  cv::Mat red_mask_1, red_mask_2, red_mask;
  laser_pts.clear();
  laser_pts.reserve(laser_ROI_.width);

  // 1. pre-processing
  cv::medianBlur(im(laser_ROI_), im_blur, 3);

  // 2. get red mask
  cv::cvtColor(im_blur, im_hsv, cv::COLOR_BGR2HSV);
  cv::inRange(im_hsv, red_mask_1_l_, red_mask_1_h_, red_mask_1);
  cv::inRange(im_hsv, red_mask_2_l_, red_mask_2_h_, red_mask_2);
  cv::bitwise_or(red_mask_1, red_mask_2, red_mask);

  cv::morphologyEx(red_mask, red_mask, cv::MORPH_DILATE, mask_dilate_kernel_);
  cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE , mask_close_kernel_);

  // 3. get masked v channel from hsv image
  cv::Mat im_hsv_masked, im_hsv_split[3];
  im_hsv.copyTo(im_hsv_masked, red_mask);
  cv::split(im_hsv_masked, im_hsv_split);
  im_v = im_hsv_split[2];

  // 4. find center of mass on each column
  std::vector<cv::Point2f> CoM_pts;
  Eigen::MatrixXd ch_v;
  cv::cv2eigen(im_v, ch_v);
  v_thresh_ = ch_v.maxCoeff() * v_thresh_ratio_;
  v_thresh_ = (v_thresh_ > 0) ? v_thresh_ : 10;
  //printf("Set thresh V: max val %f, thresh %f\n", ch_v.maxCoeff(), v_thresh_);
  findCoMColumn(ch_v, CoM_pts);

  // 5. reject outlier
  rejectOutliers(CoM_pts, laser_pts);

  return laser_pts.size() >= MIN_LASER_PTS;
}

bool LaserStripeDetector::detectLaserStripeUndistort(cv::Mat &im,
                                                     std::vector<cv::Point2f> &laser_pts) {
  assert(f_intrisics_ && "Need undistortion but intrinsics not given");

  std::vector<cv::Point2f> laser_pts_distort;
  detectLaserStripe(im, laser_pts_distort);

  cv::undistortPoints(laser_pts_distort, laser_pts, K_, D_);
  return true;
}

void LaserStripeDetector::loadEnvConfig(const std::string &env_config_fn)
{
  cv::FileStorage env_fs(env_config_fn, cv::FileStorage::READ);
  cout << "Laser stripe detector: loading from config "
       << env_config_fn << endl;
  assert(env_fs.isOpened() && "Failed to open environment config file!");
  std::vector<int> rm1l(3), rm1h(3), rm2l(3), rm2h(3), roi(4);
  env_fs["red_mask_1_l"] >> rm1l;
  env_fs["red_mask_1_h"] >> rm1h;
  env_fs["red_mask_2_l"] >> rm2l;
  env_fs["red_mask_2_h"] >> rm2h;
  env_fs["v_thresh_ratio"] >> v_thresh_ratio_;
  v_thresh_ratio_ = (v_thresh_ratio_ > 0) ? v_thresh_ratio_ : 0.1;

  red_mask_1_l_ = cv::Scalar(rm1l[0], rm1l[1], rm1l[2]);
  red_mask_1_h_ = cv::Scalar(rm1h[0], rm1h[1], rm1h[2]);
  red_mask_2_l_ = cv::Scalar(rm2l[0], rm2l[1], rm2l[2]);
  red_mask_2_h_ = cv::Scalar(rm2h[0], rm2h[1], rm2h[2]);

  env_fs["laser_ROI"] >> roi;
  int width, height;
  env_fs["image_width"] >> width;
  env_fs["image_height"] >> height;
  laser_ROI_ = cv::Rect(roi[0], roi[2], width - roi[0] - roi[1],
                        height - roi[2] - roi[3]);

  cout << "*** Laser stripe detector params ***" << endl
      << "\tmask 1 low " << red_mask_1_l_ << endl
      << "\tmask 1 high " << red_mask_1_h_ << endl
      << "\tmask 2 low " << red_mask_2_l_ << endl
      << "\tmask 2 high " << red_mask_2_h_ << endl
      << "\tlaser ROI " << laser_ROI_ << endl
      << "\tV channel thresh " << v_thresh_ratio_ << endl;
}

void LaserStripeDetector::loadCamConfig(const std::string &cam_config_fn)
{
  cv::FileStorage cam_fs(cam_config_fn, cv::FileStorage::READ);
  assert(cam_fs.isOpened() && "Failed to open camera config file!");
  cam_fs["camera_matrix"] >> K_;
  cam_fs["distortion_coefficients"] >> D_;
}

void LaserStripeDetector::initParams()
{
  mask_dilate_kernel_ = cv::Mat::ones(cv::Size(25, 50), CV_8U);
  mask_close_kernel_ = cv::Mat::ones(cv::Size(25, 35), CV_8U);
}

bool LaserStripeDetector::findCoMColumn(Eigen::MatrixXd &im,
                                        std::vector<cv::Point2f> &CoM_pts)
{
  Eigen::VectorXi col_max(im.cols());
  Eigen::VectorXi val_max(im.cols());

  Eigen::MatrixXf::Index max_index;
  for (int cc = 0; cc < im.cols(); cc++)
  {
    val_max[cc] = (int)im.col(cc).maxCoeff(&max_index);
    col_max[cc] = (int)max_index;
    if (val_max[cc] < v_thresh_) // set a low threshold on dark columns
      continue;

    int j = col_max[cc] - 1, k = col_max[cc] + 1;
    while (j >= 0        && im(j--, cc) > 0.8 * val_max[cc]);
    while (k < im.rows() && im(k++, cc) > 0.8 * val_max[cc]);

    double weighed_sum = 0., val_sum = 0.;

    for (int rr = j + 1; rr < k; rr++)
    {
      weighed_sum += im(rr, cc) * (rr - j);
      val_sum += im(rr, cc);
    }

    cv::Point2f laser_pt(cc + laser_ROI_.x,
        int(weighed_sum / val_sum + j) + laser_ROI_.y);
    CoM_pts.push_back(laser_pt);
  }

  return !CoM_pts.empty();
}

void LaserStripeDetector::rejectOutliers(const std::vector<cv::Point2f> &pts_in,
                                         std::vector<cv::Point2f> &pts_out)
{
  pts_out.clear();
  pts_out.reserve(pts_in.size());

  int seg_cnt = 0;
  for (size_t i = 0; i < pts_in.size(); i++)
  {
    size_t j = i;
    while (i != pts_in.size()
    && fabs(pts_in[i + 1].x - pts_in[i].x) < 2
    && fabs(pts_in[i + 1].y - pts_in[i].y) < 3)
      i++;

    if (i - j + 1 >= 15) // minimum number of points in a segment
    {
      seg_cnt++;
      pts_out.insert(pts_out.end(), pts_in.begin() + j, pts_in.begin() + i + 1);
    }
  }

}

bool LaserStripeDetector::setLaserExtractParams(int brightness_thresh,
                                                int hue_thresh_1,
                                                int hue_thresh_2)
{
  if (brightness_thresh < 0 || brightness_thresh > 255
      || hue_thresh_1 < 160 || hue_thresh_1 > 180
      || hue_thresh_2 < 0 || hue_thresh_2 > 20)
    return false;

  red_mask_1_h_ = cv::Scalar(180, 255, 255);
  red_mask_1_l_ = cv::Scalar(hue_thresh_1, 180, brightness_thresh);
  red_mask_2_h_ = cv::Scalar(hue_thresh_2, 255, 255);
  red_mask_2_l_ = cv::Scalar(0, 180, brightness_thresh);

  cout << "*** Changed Laser stripe detector params ***" << endl
       << "\tmask 1 low " << red_mask_1_l_ << endl
       << "\tmask 1 high " << red_mask_1_h_ << endl
       << "\tmask 2 low " << red_mask_2_l_ << endl
       << "\tmask 2 high " << red_mask_2_h_ << endl;

  return true;
}
