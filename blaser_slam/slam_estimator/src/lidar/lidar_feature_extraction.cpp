/**
 * @file lidar_feature_extraction.cpp
 *
 * @brief Adapted from SSL_SLAM2
 *
 * @date 01/05/2022
 *
 * @author Tina Tian (yutian)
 */

#include "lidar_feature_extraction.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

LidarExtractor::LidarExtractor() {}

void LidarExtractor::init(lidar::Lidar lidar_param_in)
{
    lidar_param = lidar_param_in;
}

void LidarExtractor::featureExtraction(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_edge, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_surf)
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, indices);

    // coordinate transform
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        double new_x = pc_in->points[i].z;
        double new_y = -pc_in->points[i].x;
        double new_z = -pc_in->points[i].y;
        pc_in->points[i].x = new_x;
        pc_in->points[i].y = new_y;
        pc_in->points[i].z = new_z;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> laserCloudScans;

    double last_angle = atan2(pc_in->points[0].z, pc_in->points[0].y) * 180 / M_PI;
    int count = 0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y + pc_in->points[i].z * pc_in->points[i].z);
        double angle = atan2(pc_in->points[i].x, pc_in->points[i].z) * 180 / M_PI;
        count++;

        if (fabs(angle - last_angle) > 0.05)
        {
            if (count > 30)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
                for (int k = 0; k < count; k++)
                {
                    pc_temp->push_back(pc_in->points[i - count + k + 1]);
                }
                if (pc_temp->points.size() > 0)
                    laserCloudScans.push_back(pc_temp);
            }
            count = 0;
            last_angle = angle;
        }
    }

    for (int i = 0; i < laserCloudScans.size(); i++)
    {
        std::vector<Double2d> cloudCurvature;
        int total_points = laserCloudScans[i]->points.size() - 10;
        for (int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++)
        {
            double angle_difference = fabs((atan2(laserCloudScans[i]->points[j - 5].y, laserCloudScans[i]->points[j - 5].z) - atan2(laserCloudScans[i]->points[j + 5].y, laserCloudScans[i]->points[j + 5].z)) * 180 / M_PI);
            if (angle_difference > 5)
            {
                // consider as a surf points
                pc_out_surf->push_back(laserCloudScans[i]->points[j]);
                continue;
            }

            double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10 * laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
            double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10 * laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
            double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10 * laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;
            Double2d distance(j, diffX * diffX + diffY * diffY + diffZ * diffZ);
            cloudCurvature.push_back(distance);
        }

        featureExtractionFromSector(laserCloudScans[i], cloudCurvature, pc_out_edge, pc_out_surf);
    }
}

void LidarExtractor::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_in, std::vector<Double2d> &cloudCurvature, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_edge, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out_surf)
{

    std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d &a, const Double2d &b)
              { return a.value < b.value; });

    int largestPickedNum = 0;
    std::vector<int> picked_points;
    int point_info_count = 0;
    for (int i = cloudCurvature.size() - 1; i >= 0; i--)
    {
        int ind = cloudCurvature[i].id;
        if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end())
        {
            if (cloudCurvature[i].value <= 0.1)
            {
                break;
            }

            largestPickedNum++;
            picked_points.push_back(ind);

            if (largestPickedNum <= 10)
            {
                pc_out_edge->push_back(pc_in->points[ind]);
                point_info_count++;
            }
            else
            {
                break;
            }

            for (int k = 1; k <= 5; k++)
            {
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                {
                    break;
                }
                picked_points.push_back(ind + k);
            }
            for (int k = -1; k >= -5; k--)
            {
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                {
                    break;
                }
                picked_points.push_back(ind + k);
            }
        }
    }

    for (int i = 0; i <= (int)cloudCurvature.size() - 1; i++)
    {
        int ind = cloudCurvature[i].id;
        if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end())
        {
            pc_out_surf->push_back(pc_in->points[ind]);
        }
    }
}

int LidarExtractor::findCylinder(const LidarPointCloudPtr &pc_in,
                                 LidarPointCloudPtr &pc_out,
                                 std::vector<float> &cylinder_coeff)
{
    // All the objects needed
    pcl::PassThrough<LidarPoint> pass;
    pcl::NormalEstimation<LidarPoint, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<LidarPoint, pcl::Normal> seg;
    pcl::ExtractIndices<LidarPoint> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<LidarPoint>::Ptr tree(new pcl::search::KdTree<LidarPoint>());

    // Datasets
    LidarPointCloudPtr cloud_filtered(new LidarPointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    LidarPointCloudPtr cloud_filtered2(new LidarPointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    // Build a passthrough filter to remove spurious NaNs and scene background
    // pass.setInputCloud(pc_in);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0, 1.5);
    // pass.filter(*cloud_filtered);

    copyPointCloud(*pc_in, *cloud_filtered);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
    // Extract the planar inliers from the input cloud
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    pcl::PointCloud<LidarPoint>::Ptr cloud_plane(new pcl::PointCloud<LidarPoint>());
    extract.filter(*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0.1, 0.3);
    seg.setInputCloud(cloud_filtered2);
    seg.setInputNormals(cloud_normals2);
    // Obtain the cylinder inliers and coefficients
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    // Extract the cylinder
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<LidarPoint>::Ptr cloud_cylinder(new pcl::PointCloud<LidarPoint>());
    extract.filter(*cloud_cylinder);
    if (cloud_cylinder->points.empty())
    {
        std::cerr << "Can't find the cylindrical component." << std::endl;
        return -1;
    }
    else
    {
        cylinder_coeff = coefficients_cylinder->values;
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size() << " data points." << std::endl;
        pc_out  = cloud_cylinder;
        return 0;
    }
}

Double2d::Double2d(int id_in, double value_in)
{
    id = id_in;
    value = value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in)
{
    layer = layer_in;
    time = time_in;
};
