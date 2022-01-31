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

#include "lidar_frame.h"
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

class LidarManager
{
public:
    LidarManager();

    void addLidarFrame(LidarFrameConstPtr frame);
    void discardObsoleteReadings(double time);

    int getRelativeTf(LidarFramePtr frame1, LidarFramePtr frame2, Eigen::Matrix4f &T);
    void associate(LidarPointCloudConstPtr source_cloud, 
                                LidarPointCloudConstPtr target_cloud, 
                                std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& corrs);
    void resetKDtree(LidarPointCloudConstPtr target_cloud);

private:
    std::vector<LidarFramePtr> lidar_window_;

    std::string lidar_topic;

    pcl::KdTreeFLANN<LidarPoint> kdtree;
    int K = 1;
    float dist_thresh = 0.005;

    void findVectorRot(Eigen::Vector3f &vec1, Eigen::Vector3f &vec2, Eigen::Quaternionf &q_out);

    void initialize_subscribers();
    void initialize_publishers();
    void initialize_services();
};



#endif /* LIDAR_MANAGER_H_ */