/**
 * @file lidar_frame.h
 *
 * @brief 1)
 *
 * @date 01/05/2022
 *
 * @author Tina Tian (yutian)
 */

#ifndef LIDAR_FRAME_H_
#define LIDAR_FRAME_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "lidar_feature_extraction.h"

class LidarFrame
{
public:
    LidarFrame()
    {
        axis(0) = 0;
        axis(1) = 0;
        axis(2) = 0;
        timestamp_ = 0.0;
        seq_corresp_ = 0;
    };
    explicit LidarFrame(LidarPointCloudConstPtr _pc_l,
                        double timestamp, size_t seq, int filter_num);

    double getTimestamp() const { return timestamp_; }
    Eigen::Vector3f getAxis();
    void setAxis(Eigen::Vector3f vec);

    Eigen::Vector3f getAxisPoint();

    double getRadius() const { return radius; }
    void setRadius(double rad) { radius = rad; }


    // lidar point cloud in lidar frame
    LidarPointCloudPtr pc_l_;
    LidarPointCloudPtr pc_cylinder_;
    LidarPointCloudPtr pc_edge_;
    LidarPointCloudPtr pc_surf_;

    //point_on_axis.x : the X coordinate of a point located on the cylinder axis
    //point_on_axis.y : the Y coordinate of a point located on the cylinder axis
    //point_on_axis.z : the Z coordinate of a point located on the cylinder axis
    //axis_direction.x : the X coordinate of the cylinder's axis direction
    //axis_direction.y : the Y coordinate of the cylinder's axis direction
    //axis_direction.z : the Z coordinate of the cylinder's axis direction
    //radius : the cylinder's radius
    std::vector<float> cylinder_coeff_;

    LidarExtractor lidar_extractor;
    int cylinder_extracted_ = -1;

    void get_pointcloud(LidarPointCloudPtr cloud_out);

private:
    void preproc(LidarPointCloudConstPtr pc_in, LidarPointCloudPtr pc_out);

    // transformations
    Eigen::Vector3f axis;
    double radius = 0;

    double timestamp_;
    size_t seq_corresp_; // corresponding visual image's sequence

    int point_filter_num = 100;
    const double min_dist = 0;
    const double max_dist = 100;
};

typedef std::shared_ptr<LidarFrame> LidarFramePtr;
typedef const std::shared_ptr<LidarFrame> LidarFrameConstPtr;

#endif /* LIDAR_FRAME_H_ */