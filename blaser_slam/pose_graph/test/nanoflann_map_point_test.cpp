//
// Created by dcheng on 10/29/19.
//

#include "../src/nanoflann_map_point.h"
#include "../src/utility/CameraPoseVisualization.h"
#include "../src/parameters.h"
#include "../src/utility/tic_toc.h"
#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;
int VISUALIZE_IMU_FORWARD;
int LOOP_CLOSURE;
int FAST_RELOCALIZATION;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_match_points;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_key_odometrys;
ros::Publisher pub_vio_path;
nav_msgs::Path no_loop_path;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

int main(int argc, char** argv)
{
    cout << "*** NanoFlann KD-Tree Test ***" << endl;
    PoseGraph* pg_;

    //! genearte random map points
    std::vector<MapPoint *> v_inactive_map_points_;
    Eigen::Vector3d point1(0.11302480101585388, -0.002916684839874506, -0.1072315126657486),
                    point2(0.14585094153881073,0.027723578736186028,-0.10060951858758926),
                    point3(0.14147722721099854,-0.056356940418481827,-0.10433566570281982);
    MapPoint* map_point1 = new MapPoint(point1);
    MapPoint* map_point2 = new MapPoint(point2);
    MapPoint* map_point3 = new MapPoint(point3);
    v_inactive_map_points_.push_back(map_point3);
    v_inactive_map_points_.push_back(map_point1);
    v_inactive_map_points_.push_back(map_point2);
    v_inactive_map_points_.push_back(map_point2);

    nanoflann::KDTreeMapPoint kdtree(v_inactive_map_points_);
    v_inactive_map_points_.push_back(map_point2);

    Eigen::Vector3d search_center(0.22471936047077179, 0.03105606883764267, -0.10891204327344894);

    //! search
    std::vector<int> k_indices;
    std::vector<double> k_sqr_dist;
    int n_found = kdtree.radiusSearch(search_center, 0.02, k_indices, k_sqr_dist);

    cout << "numbers found: " << n_found << endl;
    for (int i = 0; i < n_found; i++)
    {
        cout << i << "th: " << "ind " << k_indices[i] << ", dist " << k_sqr_dist[i] << endl;
    }


    return 0;
}