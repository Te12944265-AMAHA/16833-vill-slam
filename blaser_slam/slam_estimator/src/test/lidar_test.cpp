/**
 * @file lidar_test.cpp
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

    LidarPointCloudPtr cloud(new LidarPointCloud);
    LidarPointCloudPtr transformed_cloud(new LidarPointCloud);
    string filename_1 = "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/test_data/pipe_short_sample.pcd";
    string filename_2 = "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/test_data/pipe_short_sample_tf.pcd";


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

    LidarManager lidar_manager;
    LidarFramePtr frame1(new LidarFrame(cloud, 0, 0));
    LidarFramePtr frame2(new LidarFrame(transformed_cloud, 1, 0));
    cout << "frame1 cylinder: " << frame1->cylinder_extracted_ << endl;
    cout << "frame2 cylinder: " << frame2->cylinder_extracted_ << endl;

    /*
    /////////// visualize cylinder
    pcl::visualization::PCLVisualizer viewer2("cylinder");
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud_color_handler(cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer2.addPointCloud(cloud, cloud_color_handler, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud_cylinder_color_handler(frame1->pc_cylinder_, 20, 20, 230); // Red
    viewer2.addPointCloud(frame1->pc_cylinder_, cloud_cylinder_color_handler, "cylinder");

    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> transformed_cloud_color_handler(transformed_cloud, 210, 130, 130); // Red
    viewer2.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud2_cylinder_color_handler(frame2->pc_cylinder_, 20, 230, 200); // Red
    viewer2.addPointCloud(frame2->pc_cylinder_, cloud2_cylinder_color_handler, "cylinder2");

    viewer2.addCoordinateSystem(1.0, "cloud", 0);
    viewer2.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cylinder");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cylinder2");
    // viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer2.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer2.spinOnce();
    }
    */




    lidar_manager.addLidarFrame(frame1);
    lidar_manager.addLidarFrame(frame2);
    Eigen::Matrix4f tf;
    lidar_manager.getRelativeTf(frame1, frame2, tf);
    cout << "tf: " << tf << endl;
    
    // transform back
    LidarPointCloudPtr cloud2(new LidarPointCloud);
    pcl::transformPointCloud(*transformed_cloud, *cloud2, tf);
    // we don't care about the translation along the axis of the original cloud

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

    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud_color_handler(cloud, 255, 255, 255); // white
    viewer2.addPointCloud(cloud, cloud_color_handler, "cloud");

    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> transformed_cloud_color_handler(transformed_cloud, 210, 130, 130); // pink
    viewer2.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
    
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud2_color_handler(cloud2, 20, 230, 20); // green
    viewer2.addPointCloud(cloud2, cloud2_color_handler, "cloud2");

    viewer2.addCoordinateSystem(1.0, "cloud", 0);
    viewer2.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
    // viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer2.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer2.spinOnce();
    }

    return 0;
}