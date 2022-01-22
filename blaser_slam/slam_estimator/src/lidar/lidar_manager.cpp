/**
 * @file lidar_manager.cpp
 *
 * @brief 1) Subscribes to lidar topics, performs time sync.
 *        2) Request corner extraction and point cloud processing services.
 *        3) Visualize the 3D pose of the rig, accept or reject sample based on visibility.
 *        3) Creates a dataset of ground-truth and observed surfels.
 *
 * @date 01/05/2022
 *
 * @author Tina Tian (yutian)
 */

#include "lidar_manager.h"

LidarManager::LidarManager() {}

void LidarManager::addLidarFrame(LidarFrameConstPtr frame)
{
    lidar_window_.push_back(frame);
}


void LidarManager::discardObsoleteReadings(double time)
{
    auto it = lidar_window_.begin();
    while (it != lidar_window_.end() && (*it)->getTimestamp() < time)
    {
        it = lidar_window_.erase(it);
    }
}

void LidarManager::findVectorRot(Eigen::Vector3f &vec1, Eigen::Vector3f &vec2, Eigen::Quaternionf &q_out)
{
    vec1 = vec1.normalized();
    vec2 = vec2.normalized();
    double v_d = vec1.dot(vec2);
    if (v_d < 0)
        vec2 = -vec2;
    if (v_d > 0.999999) 
    {
        q_out.x() = 0;
        q_out.y() = 0;
        q_out.z() = 0;
        q_out.w() = 1;
    }
    else
    {
        q_out = Eigen::Quaternionf::FromTwoVectors(vec1, vec2);
    }
    //Eigen::Vector3f v_c = vec1.cross(vec2);
    //q.w = sqrt((v1.Length ^ 2) * (v2.Length ^ 2)) + dotproduct(v1, v2);
}

int LidarManager::getRelativeTf(LidarFramePtr frame1, LidarFramePtr frame2,
                                Eigen::Matrix4f &T)
{
    if (frame1 == nullptr || frame2 == nullptr || frame1->cylinder_extracted_ == -1 || frame2->cylinder_extracted_ == -1)
    {
        std::cerr << "Cannot compute relative Tf between lidar frames because some frames are invalid" << std::endl;
        return -1;
    }
    Eigen::Quaternionf dq;
    Eigen::Vector3f dt;

    Eigen::Vector3f vec1 = frame1->getAxis();
    Eigen::Vector3f vec2 = frame2->getAxis();
    // 1. straighten frame 1 cylinder and apply the same tf to frame 2
    Eigen::Vector3f vecz(0.0, 0.0, 1.0);
    Eigen::Quaternionf dq_vec1;
    findVectorRot(vec1, vecz, dq_vec1);
    Eigen::Matrix3f dR_vec1 = dq_vec1.normalized().toRotationMatrix();

    // 2. compute R_0_1
    // dq brings v2 to v1
    findVectorRot(vec2, vec1, dq);
    frame1->setAxis(vec1);
    frame2->setAxis(vec2);
    // R_1_0:
    Eigen::Matrix3f R = dq.normalized().toRotationMatrix().transpose();

    // 3. Compute translation by: R_1_0 * p0 = t_1_2 * p2
    Eigen::Vector3f p0 = frame1->getAxisPoint();
    Eigen::Vector3f p2 = frame2->getAxisPoint();
    Eigen::Vector3f p0_prime = dR_vec1 * p0;
    Eigen::Vector3f p2_prime = dR_vec1 * p2;
    Eigen::Vector3f p1_prime = R * p0_prime;
    dt = p1_prime - p2_prime;

    // 4. assemble the transformation matrix
    Eigen::Matrix3f Rdq = dq.normalized().toRotationMatrix();
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++) {
            tf(i, j) = Rdq(i, j);
        }
    }
    Eigen::Matrix4f Tdt = Eigen::Matrix4f::Identity();
    Tdt(0, 3) = dt.x();
    Tdt(1, 3) = dt.y();
    Tdt(2, 3) = dt.z();
    // first translate to align the starting point, then rotate back
    tf = tf * Tdt;
    tf(2, 3) = 0; // set z trans to 0
    T = tf;
    return 0;
}