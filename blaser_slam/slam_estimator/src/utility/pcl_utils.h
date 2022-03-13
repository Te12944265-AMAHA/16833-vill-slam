/**
 * @file pcd_utils.h
 *
 * @brief Utility functions for point cloud
 *
 * @date 02/26/2022
 *
 * @author Tina Tian (yutian)
 */

#ifndef PCD_UTILS_H_
#define PCD_UTILS_H_

#include <stdio.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "../lidar/lidar.h"


void cloud_msg_to_pcl(sensor_msgs::PointCloud2ConstPtr msg_in, LidarPointCloudPtr pcl_p_out);

void pcl_to_cloud_msg(LidarPointCloudConstPtr pcl_in, sensor_msgs::PointCloud2Ptr msg_out);


#endif /* PCD_UTILS_H_ */
