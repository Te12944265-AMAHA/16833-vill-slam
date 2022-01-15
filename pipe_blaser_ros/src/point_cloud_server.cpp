/*
* point_cloud_server.cpp
* Listen to the point cloud topic and send data via a socket
* For AR applications
*
* Created by Luyuan Wang
*/

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>

#include <third_party/async-sockets/tcpserver.hpp>

#include "std_msgs/String.h"

using namespace std;

static TCPServer tcpServer; // use static variables to prevent releasing tcpServer instance... which will cause seg fault
static TCPSocket *clientSocket;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void pointCloudPCLCallback(PointCloud pc) {
    std::stringstream buffer;

    for (pcl::PointXYZRGB point : pc.points) {
        float x = point.x;
        float y = point.y;
        float z = point.z;

        std::uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
        std::uint8_t r = (rgb >> 16) & 0x0000ff;
        std::uint8_t g = (rgb >> 8)  & 0x0000ff;
        std::uint8_t b = (rgb)       & 0x0000ff;

        // TODO: a simple encoding method, should be replaced for better efficiency
        buffer  << x << " " << y << " " << z << " " << (float)r << " " << (float)g << " "  << (float)b << ";";
    }

    cout << "Sending point cloud..." << endl;
    if (clientSocket) {
        clientSocket->Send(buffer.str());
    }
}


void startServer() {
    // When a new client connected
    tcpServer.onNewConnection = [&](TCPSocket *newClient) {
        cout << "New client: [";
        cout << newClient->remoteAddress() << ":" << newClient->remotePort() << "]" << endl;
        clientSocket = newClient;
    };

    // Bind the server to a port
    tcpServer.Bind(8888, [](int errorCode, string errorMessage) {
        // BINDING FAILED:
        cout << errorCode << " : " << errorMessage << endl;
    });

    // Start Listening the server
    tcpServer.Listen([](int errorCode, string errorMessage) {
        // LISTENING FAILED:
        cout << errorCode << " : " << errorMessage << endl;
    });
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_server");

    std::cout << "Point Cloud Server starting..." << std::endl;

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/slam_estimator/laser_pcd", 100, pointCloudPCLCallback);

    startServer();

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}