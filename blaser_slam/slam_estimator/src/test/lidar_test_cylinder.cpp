/**
 * @file lidar_test_cylinder.cpp
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

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud0(new pcl::PointCloud<pcl::PointXYZRGB>);
    LidarPointCloudPtr cloud(new LidarPointCloud);
    LidarPointCloudPtr transformed_cloud(new LidarPointCloud);
    string filename_1 = "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/test_data/mapped.pcd";
    string filename_2 = "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/test_data/lidar.pcd";


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


    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename_1, *cloud0) == -1 
        || pcl::io::loadPCDFile<LidarPoint>(filename_2, *transformed_cloud) == -1)
    {
        cout << "Couldn't read pcd file." << endl;
        return -1;
    }

    copyPointCloud(*cloud0, *cloud);
    cout << cloud->points.size() << endl;

    LidarManager lidar_manager;
    LidarFramePtr frame1(new LidarFrame(cloud, 5));
    LidarFramePtr frame2(new LidarFrame(transformed_cloud, 100));
    cout << "frame1 #points: " << frame1->pc_l_->size() << endl;
    cout << "frame2 #points: " << frame2->pc_l_->size() << endl;
    cout << "frame1 cylinder: " << frame1->extractCylinder() << endl;
    cout << "frame2 cylinder: " << frame2->extractCylinder() << endl;

    Eigen::Vector3f p1 = frame1->getAxisPoint();
    Eigen::Vector3f d1 = frame1->getAxis();
    auto p1s = p1 + 4.5*d1;
    auto p1e = p1s - 1.5* d1;
    LidarPoint p1s_l(p1s[0], p1s[1], p1s[2]);
    LidarPoint p1e_l(p1e[0], p1e[1], p1e[2]);

    Eigen::Vector3f p2 = frame2->getAxisPoint();
    Eigen::Vector3f d2 = frame2->getAxis();
    auto p2s = p2 - 1.5*d2;
    auto p2e = p2s - 2*d2;
    LidarPoint p2s_l(p2s[0], p2s[1], p2s[2]);
    LidarPoint p2e_l(p2e[0], p2e[1], p2e[2]);

  /*  
    /////////// visualize cylinder
    pcl::visualization::PCLVisualizer viewer2("cylinder");
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud_color_handler(cloud, 80, 70, 242);
    // We add the point cloud to the viewer and pass the color handler
    viewer2.addPointCloud(cloud, cloud_color_handler, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud_cylinder_color_handler(frame1->pc_cylinder_, 20, 20, 230); // Red
    //viewer2.addPointCloud(frame1->pc_cylinder_, cloud_cylinder_color_handler, "cylinder");

    viewer2.addArrow(p1s_l, p1e_l, 80, 70, 242, false, "arr");

    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> transformed_cloud_color_handler(transformed_cloud, 242, 70, 80); // Red
    viewer2.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud2_cylinder_color_handler(frame2->pc_cylinder_, 20, 230, 200); // Red
    //viewer2.addPointCloud(frame2->pc_cylinder_, cloud2_cylinder_color_handler, "cylinder2");
    
    viewer2.addArrow(p2e_l, p2s_l, 242, 70, 80, false, "arro");

    viewer2.addCoordinateSystem(0.3, "cloud", 0);
    viewer2.setBackgroundColor(0.05, 0, 0, 0); // Setting background to white grey

    viewer2.initCameraParameters();
    viewer2.setCameraPosition(1, 2, -1,    0, 0, 1,   -0.1, 0.1, -0.25);
    viewer2.setCameraFieldOfView(0.523599);
    viewer2.setCameraClipDistances(0.00522511, 50); 

    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    //viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cylinder");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud");
    //viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cylinder2");
    //viewer2.setPosition(800, 400); // Setting visualiser window position
    //viewer2.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "arr");
    //viewer2.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "arro");


    while (!viewer2.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer2.spinOnce();
    }
    
*/


    cout << "done axes extraction"<< endl;
    Eigen::Matrix4f tf;
    lidar_manager.getRelativeTf(frame1, frame2, tf);
    cout << "relative tf: " << tf << endl;
    
    // transform back
    LidarPointCloudPtr cloud2(new LidarPointCloud);
    pcl::transformPointCloud(*transformed_cloud, *cloud2, tf);
    // we don't care about the translation along the axis of the original cloud
/*
    cout <<" gt: " << endl;
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 0.2, 0.1, 0.0;
    // rotate around x axis
    float theta = M_PI / 6;
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
    // Print the transformation
    std::cout << transform_2.matrix() << std::endl;
*/


    /////////// visualize
    pcl::visualization::PCLVisualizer viewer2("cylinder");

    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud_color_handler(cloud, 80, 70, 242); // white
    viewer2.addPointCloud(cloud, cloud_color_handler, "cloud");

    viewer2.addArrow(p1s_l, p1e_l, 80, 70, 242, false, "arr");
    
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud2_color_handler(cloud2, 242, 70, 80); // green
    viewer2.addPointCloud(cloud2, cloud2_color_handler, "cloud2");

    Eigen::Vector4f p2ss(p2s[0], p2s[1], p2s[2], 1);
    Eigen::Vector4f p2ee(p2e[0], p2e[1], p2e[2], 1);
    auto p2s_t = tf * p2ss;
    auto p2e_t = tf * p2ee;
    LidarPoint p2s_lt(p2s_t[0], p2s_t[1], p2s_t[2]);
    LidarPoint p2e_lt(p2e_t[0], p2e_t[1], p2e_t[2]);
    viewer2.addArrow(p2e_lt, p2s_lt, 242, 70, 80, false, "arro");

    viewer2.addCoordinateSystem(0.3, "cloud", 0);
    viewer2.setBackgroundColor(0.05, 0, 0, 0); // Setting background to white grey

    viewer2.initCameraParameters();
    viewer2.setCameraPosition(1, 2, -1,    0, 0, 1,   -0.1, 0.1, -0.25);
    viewer2.setCameraFieldOfView(0.523599);
    viewer2.setCameraClipDistances(0.00522511, 50);

    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
    // viewer.setPosition(800, 400); // Setting visualiser window position
    viewer2.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "arr");
    viewer2.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "arro");

    while (!viewer2.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer2.spinOnce();
    }

    return 0;
}