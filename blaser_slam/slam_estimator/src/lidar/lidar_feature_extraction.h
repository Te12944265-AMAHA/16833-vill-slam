/**
 * @file lidar_feature_extraction.h
 *
 * @brief Adapted from SSL_SLAM2
 *
 * @date 01/05/2022
 *
 * @author Tina Tian (yutian)
 */


#ifndef LIDAR_EXTRACTION_H_
#define LIDAR_EXTRACTION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include "lidar.h"
#include <ros/ros.h>

//points covariance class
class Double2d{
public:
	int id;
	double value;
	Double2d(int id_in, double value_in);
};
//points info class
class PointsInfo{
public:
	int layer;
	double time;
	PointsInfo(int layer_in, double time_in);
};

class LidarExtractor 
{
    public:
    	LidarExtractor();

		void init(lidar::Lidar lidar_param_in);

		void featureExtraction(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, 
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_edge, 
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_surf);

		void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, 
                                         std::vector<Double2d>& cloudCurvature, 
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_edge, 
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_surf);	

        int findCylinder(const LidarPointCloudPtr &pc_in, 
                               LidarPointCloudPtr &pc_out,
							   std::vector<float> &cylinder_coeff);
	private:
     	lidar::Lidar lidar_param;
};



#endif // LIDAR_EXTRACTION_H_

