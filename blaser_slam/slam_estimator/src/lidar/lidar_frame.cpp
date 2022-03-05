/**
 * @file lidar_frame.cpp
 *
 * @brief 1)
 *
 * @date 01/05/2022
 *
 * @author Tina Tian (yutian)
 */

#include "lidar_frame.h"

LidarFrame::LidarFrame(LidarPointCloudConstPtr _pc_l, int filter_num)
    : pc_l_(new LidarPointCloud), point_filter_num(filter_num),
      pc_cylinder_(new LidarPointCloud), pc_edge_(new LidarPointCloud), pc_surf_(new LidarPointCloud)
{
    axis(0) = 0;
    axis(1) = 0;
    axis(2) = 0;
    preproc(_pc_l, pc_l_);
    // extract features
    //lidar_extractor.featureExtraction(pc_l_, pc_edge, pc_surf);
    //cylinder_extracted_ = lidar_extractor.findCylinder(pc_l_, pc_cylinder_, cylinder_coeff_);
}

LidarPointCloudPtr LidarFrame::get_pointcloud()
{
    LidarPointCloudPtr cloud_out;
    *cloud_out = *pc_l_;
    return cloud_out;
}

Eigen::Vector3f LidarFrame::getAxis()
{
    if (axis(0) == 0 && axis(1) == 0 && axis(2) == 0)
    {
        Eigen::Vector3f res(cylinder_coeff_[3], cylinder_coeff_[4], cylinder_coeff_[5]);
        return res;
    }
    else 
        return axis;
}

Eigen::Vector3f LidarFrame::getAxisPoint()
{
    Eigen::Vector3f res(cylinder_coeff_[0], cylinder_coeff_[1], cylinder_coeff_[2]);
    return res;
}

void LidarFrame::setAxis(Eigen::Vector3f vec)
{
    axis = vec;
}

void LidarFrame::preproc(LidarPointCloudConstPtr pc_in, LidarPointCloudPtr pc_out)
{
    //! 1. subsample
    pc_out->clear();

    for (int i = 0; i < pc_in->points.size(); i++)
    {
        if (i % point_filter_num != 0)
            continue;
        double range = pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y + pc_in->points[i].z * pc_in->points[i].z;
        if (range < (min_dist * min_dist) || range > (max_dist * max_dist))
            continue;
        pc_out->points.push_back(pc_in->points[i]);
    }
}