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

void LidarManager::findVectorRot(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2, Eigen::Quaterniond &q_out)
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
        q_out = Eigen::Quaterniond::FromTwoVectors(vec1, vec2);
    }
    //Eigen::Vector3d v_c = vec1.cross(vec2);
    //q.w = sqrt((v1.Length ^ 2) * (v2.Length ^ 2)) + dotproduct(v1, v2);
}

int LidarManager::getRelativeTf(LidarFramePtr frame1, LidarFramePtr frame2,
                                Eigen::Quaterniond &dq, Eigen::Vector3d &dt)
{
    if (frame1 == nullptr || frame2 == nullptr || frame1->cylinder_extracted_ == -1 || frame2->cylinder_extracted_ == -1)
    {
        std::cerr << "Cannot compute relative Tf between lidar frames because some frames are invalid" << std::endl;
        return -1;
    }
    Eigen::Vector3d vec1 = frame1->getAxis();
    Eigen::Vector3d vec2 = frame2->getAxis();
    // dq brings v2 to v1
    findVectorRot(vec2, vec1, dq);
    frame1->setAxis(vec1);
    frame2->setAxis(vec2);
    // set axis2 parallel to axis1 and compute dt as the line-to-line translation in frame2
    dt(0) = frame1->cylinder_coeff_[0] - frame2->cylinder_coeff_[0];
    dt(1) = frame1->cylinder_coeff_[1] - frame2->cylinder_coeff_[1];
    dt(2) = frame1->cylinder_coeff_[2] - frame2->cylinder_coeff_[2];
    return 0;
}