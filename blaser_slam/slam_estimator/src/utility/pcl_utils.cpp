/**
 * @file pcd_utils.cpp
 *
 * @brief Utility functions for point cloud
 *
 * @date 02/26/2022
 *
 * @author Tina Tian (yutian)
 */

#include "pcl_utils.h"

void cloud_msg_to_pcl(sensor_msgs::PointCloud2ConstPtr msg_in, LidarPointCloudPtr pcl_p_out)
{
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*msg_in, *pcl_pc2);
    pcl::fromPCLPointCloud2(*pcl_pc2, *pcl_p_out);
}