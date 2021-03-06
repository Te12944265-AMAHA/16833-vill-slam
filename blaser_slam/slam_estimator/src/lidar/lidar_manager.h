/**
 * @file data_manager.h
 *
 * @brief 1)
 *        2) Request corner extraction and point cloud processing services.
 *        3) Visualize the 3D pose of the rig, accept or reject sample based on visibility.
 *        3) Creates a dataset of ground-truth and observed surfels.
 *
 * @date 07/26/2021
 *
 * @author Tina Tian (yutian)
 */

#ifndef LIDAR_MANAGER_H_
#define LIDAR_MANAGER_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

#include <ceres/ceres.h>
#include "lidar_frame.h"
#include "../factor/lidar_factor.h"
#include "../utility/geometry_utils.h"
#include <Eigen/Dense>

#include "../parameters.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "../factor/pose_local_parameterization.h"

typedef struct lidar_data_frame
{
    double feature_time;
    bool use_interpolate;
    LidarFramePtr f1_p;
    //LidarFramePtr f2_p;
    Eigen::Matrix4f T_cur_1;
    //Eigen::Matrix4f T_cur_2;
} LidarDataFrame;

typedef std::shared_ptr<LidarDataFrame> LidarDataFramePtr;
typedef const std::shared_ptr<LidarDataFrame> LidarDataFrameConstPtr;

class LidarManager
{
public:
    LidarManager();
    int getWindowSize() { return lidar_window_.size(); }
    void addLidarDataFrame(LidarDataFrameConstPtr frame, double stamp);

    int get_lidar_frame(const double time, LidarDataFramePtr frame_out) const;

    void discardObsoleteReadings(double time);

    int getRelativeTf(LidarFramePtr frame1, LidarFramePtr frame2, Eigen::Matrix4f &T);
    int getRelativeTf(double t1, double t2, Eigen::Matrix4f &T);

    void associate(LidarPointCloudConstPtr source_cloud, 
                                LidarPointCloudConstPtr target_cloud, 
                                std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& corrs);
    void align(LidarPointCloudConstPtr source_cloud,
               LidarPointCloudConstPtr target_cloud,
               std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &corrs,
               Eigen::Affine3f &tf_out);

    void align_pcl_icp(LidarPointCloudConstPtr source_cloud,
               LidarPointCloudConstPtr target_cloud,
               std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &corrs,
               Eigen::Affine3f &tf_out);

    void get_tf_between_data_frames(LidarDataFrameConstPtr df1, 
                                    LidarDataFrameConstPtr df2,  
                                    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &corrs_df1f2_df2f1,
                                    Eigen::Affine3f &tf_out);
    
    int get_relative_tf(double t1,
                           double t2, 
                           std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &corrs,
                           Eigen::Matrix4d &T_prev_2,
                           Eigen::Matrix4d &T_cur_1);

    void resetKDtree(LidarPointCloudConstPtr target_cloud);

private:
    // maps feature time to data frame
    std::map<double, LidarDataFrameConstPtr> lidar_window_;

    std::string lidar_topic;

    pcl::KdTreeFLANN<LidarPoint> kdtree;
    int K = 1;
    float dist_thresh = 10;

    void findVectorRot(Eigen::Vector3f &vec1, Eigen::Vector3f &vec2, Eigen::Quaternionf &q_out);

    void initialize_subscribers();
    void initialize_publishers();
    void initialize_services();
};



#endif /* LIDAR_MANAGER_H_ */