/**
 * @file lidar_test_icp.cpp
 *
 * @brief 1) Read in point cloud, process it into a rotated and translated version, save
 *        2) Find transformation between the two clouds
 *        3) Visualize the clouds
 *
 * @date 01/21/2021
 *
 * @author Tina Tian (yutian)
 */

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../lidar/lidar_frame.h"
#include "../lidar/lidar_manager.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{

    // test pcl reserve
    LidarPointCloudPtr pc_tmp (new LidarPointCloud);
    pc_tmp->reserve(4);
    for (int i = 0; i < 3; i++)
    {
        LidarPoint pt_tmp;
        pt_tmp.x = 1.1;
        pt_tmp.y = 2.2;
        pt_tmp.z = 3.3;
        pc_tmp->push_back(pt_tmp);
    }
    cout << "size: " << pc_tmp->size() << endl;
    for (int i = 0; i <  pc_tmp->size(); i++) 
    {
        cout << pc_tmp->points[i] << endl;
    }

    exit(0);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud0(new pcl::PointCloud<pcl::PointXYZRGB>);
    LidarPointCloudPtr cloud(new LidarPointCloud);
    LidarPointCloudPtr transformed_cloud(new LidarPointCloud);
    string filename_1 = "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/test_data/pipe_short_sample.pcd";
    string filename_2 = "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/test_data/pipe_short_sample_tf.pcd";

/*
    double pose[7] = {1,2,3,4,5,6,7}; // the transformation T such that dst = T * src
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t(pose);
    Eigen::Map<Eigen::Quaternion<double>> q(pose + 3);
    t << 0, 0, 0;
    q = Eigen::Quaterniond::Identity();
    for (int i = 0; i < 7; i++){
        cout << pose[i] << ", ";
    }
    cout << endl;
    return 0;
*/
    // Read in the cloud data
    /*
    if (pcl::io::loadPCDFile<LidarPoint>(filename_1, *cloud) == -1)
    {
        cout << "Couldn't read pcd file." << endl;
        return -1;
    }
    cout << "PointCloud has: " << cloud->size() << " data points." << endl;

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 0.2, 0.1, 0.0;
    // rotate around x axis
    float theta = M_PI / 6;
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
    // Print the transformation
    std::cout << transform_2.matrix() << std::endl;

    // Executing the transformation
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);

    pcl::io::savePCDFileASCII (filename_2, *transformed_cloud);
    */


    if (pcl::io::loadPCDFile<LidarPoint>(filename_1, *cloud) == -1 
        || pcl::io::loadPCDFile<LidarPoint>(filename_2, *transformed_cloud) == -1)
    {
        cout << "Couldn't read pcd file." << endl;
        return -1;
    }

    cout << "original cloud size: " << cloud->points.size() << endl;

    LidarManager lidar_manager;
    LidarFramePtr frame1(new LidarFrame(cloud, 10));
    LidarFramePtr frame2(new LidarFrame(transformed_cloud, 10));

    vector<pair<Eigen::Vector3f, Eigen::Vector3f>> corrs;
    Eigen::Affine3f tf;

    cloud = frame1->get_pointcloud();
    cout << "preprocessed cloud size: " << cloud->points.size() << endl;

    lidar_manager.align_pcl_icp(frame2->pc_l_, frame1->pc_l_, corrs, tf);


    
    // transform back
    LidarPointCloudPtr cloud2(new LidarPointCloud);
    pcl::transformPointCloud(*transformed_cloud, *cloud2, tf);
    // to matrix
    cout << "tf: " << endl << tf.matrix() << endl;

    cout <<" gt: " << endl;
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 0.2, 0.1, 0.0;
    // rotate around x axis
    float theta = M_PI / 6;
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
    // Print the transformation
    std::cout << transform_2.matrix() << std::endl;



    /////////// visualize
    pcl::visualization::PCLVisualizer viewer2("cylinder");

    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud_color_handler(cloud, 80, 70, 242); // white
    viewer2.addPointCloud(cloud, cloud_color_handler, "cloud");
  
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud2_color_handler(cloud2, 242, 70, 80); // green
    viewer2.addPointCloud(cloud2, cloud2_color_handler, "cloud2");

    viewer2.addCoordinateSystem(0.3, 0);
    viewer2.setBackgroundColor(0.05, 0, 0, 0); // Setting background to white grey
/*
    viewer2.initCameraParameters();
    viewer2.setCameraPosition(1, 2, -1,    0, 0, 1,   -0.1, 0.1, -0.25);
    viewer2.setCameraFieldOfView(0.523599);
    viewer2.setCameraClipDistances(0.00522511, 50);
*/
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");

    while (!viewer2.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer2.spinOnce();
    }

    return 0;
}