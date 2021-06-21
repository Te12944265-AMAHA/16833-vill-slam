//
// Created by dcheng on 3/22/20.
//

#include <blaser_ros/laser_stripe_detector.h>
#include <opencv2/core/eigen.hpp>

using std::cout;
using std::endl;
LaserStripeDetector::LaserStripeDetector(const std::string &config_fn)
: f_intrisics_(false)
, val_min_(0.0)
{
  initParams();
  loadEnvConfig(config_fn);
}

LaserStripeDetector::LaserStripeDetector(const std::string &env_config_fn,
                                         const std::string &cam_config_fn)
: f_intrisics_(true)
{
  initParams();
  loadEnvConfig(env_config_fn);
  loadCamConfig(cam_config_fn);
}

bool LaserStripeDetector::detectLaserStripe(cv::Mat &im,
                                            std::vector<cv::Point2f> &laser_pts) {
  cv::Mat im_blur, im_hsv, im_v;
  cv::Mat hsv_mask;
  laser_pts.clear();
  laser_pts.reserve(laser_ROI_.width);

  // 1. pre-processing
  cv::medianBlur(im(laser_ROI_), im_blur, 3);

  // 2. get red mask
  cv::cvtColor(im_blur, im_hsv, cv::COLOR_BGR2HSV);
  generateHSVMasks(im_hsv, hsv_mask);

  cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_DILATE, mask_dilate_kernel_);
  cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_CLOSE , mask_close_kernel_);

  // 3. get masked v channel from hsv image
  cv::Mat im_hsv_masked, im_hsv_split[3];
  im_hsv.copyTo(im_hsv_masked, hsv_mask);
  cv::split(im_hsv_masked, im_hsv_split);
  im_v = im_hsv_split[2];

  // 4. find center of mass on each column
  std::vector<cv::Point2f> CoM_pts;
  Eigen::MatrixXd ch_v;
  cv::cv2eigen(im_v, ch_v);
  val_min_ = ch_v.maxCoeff() * val_ratio_;
  val_min_ = (val_min_ > 0) ? val_min_ : 10;
  //printf("Set thresh V: max val %f, thresh %f\n", ch_v.maxCoeff(), val_min_);
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
  //std::vector<int> rm1l(3), rm1h(3), rm2l(3), rm2h(3)
  std::vector<int> roi(4);
  env_fs["hue_min"] >> hue_min_;
  env_fs["hue_max"] >> hue_max_;
  env_fs["sat_min"] >> sat_min_;
  env_fs["val_min"] >> val_min_;
  env_fs["val_ratio"] >> val_ratio_;
  val_ratio_ = (val_ratio_ > 0) ? val_ratio_ : 0.1;

  env_fs["laser_ROI"] >> roi;
  int width, height;
  env_fs["image_width"] >> width;
  env_fs["image_height"] >> height;
  laser_ROI_ = cv::Rect(roi[0], roi[2], width - roi[0] - roi[1],
                        height - roi[2] - roi[3]);

  cout << "*** Laser stripe detector params ***" << endl
      << "\tmask hue min " << hue_min_ << endl
      << "\tmask hue max " << hue_max_ << endl
      << "\tmask sat min " << sat_min_ << endl
      << "\tmask val min " << val_min_ << endl
      << "\tlaser ROI " << laser_ROI_ << endl
      << "\tval ratio " << val_ratio_ << endl;
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
    if (val_max[cc] < val_min_) // set a low threshold on dark columns
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

bool LaserStripeDetector::setLaserExtractParams(int val_min,
                                                int hue_min,
                                                int hue_max)
{
  if (val_min < 0 || val_min > 255
      || hue_min < 0 || hue_min > 180
      || hue_max < 0 || hue_max > 180)
    return false;

  val_min_ = val_min;
  hue_min_ = hue_min;
  hue_max_ = hue_max;

  cout << "*** Changed Laser stripe detector params ***" << endl
       << "\tmask hue min " << hue_min_ << endl
       << "\tmask hue max " << hue_max_ << endl
       << "\tmask val min " << val_min_ << endl;

  return true;
}

void LaserStripeDetector::generateHSVMasks(const cv::Mat& im_hsv, cv::Mat& hsv_mask) const
{
  // get max value (intensity) of image
  cv::Mat v_channel;
  cv::extractChannel(im_hsv, v_channel, 2);
  double val_max, tmp;
  cv::minMaxIdx(v_channel, &tmp, &val_max);

  // compute hsv mask
  double val_min = std::max(val_min_, val_max * val_ratio_);
  if (hue_min_ > hue_max_) // red color that covers hue value = 180/0
  {
    cv::Mat hsv_mask_1, hsv_mask_2;
    cv::inRange(im_hsv, cv::Scalar(0       , sat_min_, val_min),
                        cv::Scalar(hue_max_, 255     , 255    ), hsv_mask_1);
    cv::inRange(im_hsv, cv::Scalar(hue_min_, sat_min_, val_min),
                        cv::Scalar(180     , 255     , 255    ), hsv_mask_2);
    cv::bitwise_or(hsv_mask_1, hsv_mask_2, hsv_mask);
  }
  else
  {
    cv::inRange(im_hsv, cv::Scalar(hue_min_, sat_min_, val_min),
                        cv::Scalar(hue_max_, 255     , 255    ), hsv_mask);
  }
}
