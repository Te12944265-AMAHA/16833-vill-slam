#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "parameters.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"

#define MIN_LOOP_NUM 25

using namespace Eigen;
using namespace std;
using namespace DVision;


class BriefExtractor
{
public:
  virtual void operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const;
  BriefExtractor(const std::string &pattern_file);

  DVision::BRIEF m_brief;
};

class KeyFrame
{
public:
	KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
			 vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_normal, 
			 vector<double> &_point_id, int _sequence);
	KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
			 cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1 > &_loop_info,
			 vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm, vector<BRIEF::bitset> &_brief_descriptors);

	~KeyFrame();

	/**
	 * Try to find geometric (2D-3D) relationship between current frame and the loop frame
	 * 1. Search for feature correspondences between two frames with BRIEF descriptor
	 * 2. If have enough feature correspondences, do RANSAC + PnP to remove outliers
	 * 3. If have enough inliers, load up current frame's loop closure information (bool has_loop, matched frame id,
	 *    relative translation and rotation, etc).
	 * 4. Publish matched points for fast vins_estimator relocalization. todo: why not publish transformation correction?
	 * @param old_kf the pointer to the detected loop frame
	 * @return false if can't find enough inliers after RANSAC + PnP
	 */
	bool findConnection(KeyFrame* old_kf);

	/**
	 * Compute BRIEF descriptor of the feature points that came in the sliding window
	 */
	void computeWindowBRIEFPoint();

	/**
	 * Compute BRIEF descriptor for FAST corners, used for loop closure detection
	 */
	void computeBRIEFPoint();
	//void extractBrief();
	int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);
	bool searchInAera(const BRIEF::bitset window_descriptor,
	                  const std::vector<BRIEF::bitset> &descriptors_old,
	                  const std::vector<cv::KeyPoint> &keypoints_old,
	                  const std::vector<cv::KeyPoint> &keypoints_old_norm,
	                  cv::Point2f &best_match,
	                  cv::Point2f &best_match_norm);
	void searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
						  std::vector<cv::Point2f> &matched_2d_old_norm,
                          std::vector<uchar> &status,
                          const std::vector<BRIEF::bitset> &descriptors_old,
                          const std::vector<cv::KeyPoint> &keypoints_old,
                          const std::vector<cv::KeyPoint> &keypoints_old_norm);
	void FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                                const std::vector<cv::Point2f> &matched_2d_old_norm,
                                vector<uchar> &status);

	/**
	 * Perform PnP RANSAC between 2d image points in old camera frame to 3d current key points in world frame
	 * @param matched_2d_old_norm 2d image normalized positions in old camera frame
	 * @param matched_3d 3d current key points in world frame
	 * @param status which ones are valid
	 * @param PnP_T_old
	 * @param PnP_R_old
	 */
	void PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
	               const std::vector<cv::Point3f> &matched_3d,
	               std::vector<uchar> &status,
	               Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old);
	void getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
	void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
	void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
	void updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
	void updateLoop(Eigen::Matrix<double, 8, 1 > &_loop_info);

	Eigen::Vector3d getLoopRelativeT();
	double getLoopRelativeYaw();
	Eigen::Quaterniond getLoopRelativeQ();

    void saveImage();
    void loadImage(cv::Mat &image);

	double time_stamp; 
	int index;
	int local_index;

	// vio pose: initially set to incoming messages from vio, only changed when it belongs to different sequence
	//           from the world frame sequence
	Eigen::Vector3d vio_T_w_i; 
	Eigen::Matrix3d vio_R_w_i;

	// optimized pose: updated by optimization
	Eigen::Vector3d T_w_i;
	Eigen::Matrix3d R_w_i;

	// origin vio pose: the original vio pose set by incoming messages from vio, never changed
	Eigen::Vector3d origin_vio_T;		
	Eigen::Matrix3d origin_vio_R;
	cv::Mat image;
	cv::Mat thumbnail;
	vector<cv::Point3f> point_3d; // 3d point positions of the keyframe in WORLD frame
	vector<cv::Point2f> point_2d_uv;
	vector<cv::Point2f> point_2d_norm;
	vector<double> point_id;
	vector<cv::KeyPoint> keypoints;
	vector<cv::KeyPoint> keypoints_norm;
	vector<cv::KeyPoint> window_keypoints;
	vector<BRIEF::bitset> brief_descriptors;
	vector<BRIEF::bitset> window_brief_descriptors;
	bool has_fast_point;
	int sequence;

	bool has_loop;
	int loop_index;

	/**
	 * loop info contains three translation elements, four quaternion elements, and one yaw element
	 * 1. relative translation in the loop closure frame: R_w_old * (p_w_cur - p_w_old)
	 * 2. relative rotation in the loop closure frame: R_old_w * R_w_cur
	 * 3. yaw in world sense: yaw of cur - yaw of old
	 */
	Eigen::Matrix<double, 8, 1 > loop_info;
};

