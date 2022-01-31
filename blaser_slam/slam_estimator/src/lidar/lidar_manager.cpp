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
    // Eigen::Vector3f v_c = vec1.cross(vec2);
    // q.w = sqrt((v1.Length ^ 2) * (v2.Length ^ 2)) + dotproduct(v1, v2);
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
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
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

void LidarManager::resetKDtree(LidarPointCloudConstPtr target_cloud)
{
    kdtree.setInputCloud(target_cloud);
    /*
    // build target cloudpoints kdtree
    size_t N = m_stCloudPoints_target->size();
    m_KDtree_pts_target.pts.clear();
    m_KDtree_pts_target.pts.resize(N);
    for(size_t i = 0; i < N; i++){
        LidarPoint pt = m_stCloudPoints_target.at(i);
        m_KDtree_pts_target.pts.at(i).x = pt.x;
        m_KDtree_pts_target.pts.at(i).y = pt.y;
    }

    m_pKDtree_target.reset(new kd_tree_t(2, m_KDtree_pts_target, nanoflann::KDTreeSingleIndexAdaptorParams(1)));
    m_pKDtree_target->buildIndex();
    */
}

void LidarManager::associate(LidarPointCloudConstPtr source_cloud, 
                                LidarPointCloudConstPtr target_cloud, 
                                std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& corrs)
{

    for (int i = 0; i < source_cloud->size(); i++)
    {
        LidarPoint searchPoint = source_cloud->points[i];
        std::vector<int> pointIdxKNNSearch(K);
        std::vector<float> pointKNNSquaredDistance(K);
        //float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
        if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
        {
            if (pointKNNSquaredDistance[0] < dist_thresh) {
                LidarPoint pt = (*target_cloud)[pointIdxKNNSearch[0]];
                Eigen::Vector3f p1(pt.x, pt.y, pt.z);
                Eigen::Vector3f p2(searchPoint.x, searchPoint.y, searchPoint.z);
                corrs.push_back(std::make_pair(p1, p2));
            }
        }

    }

    /*
    m_vAssociation.clear();
    for(size_t i = 0; i < m_stCloudPoints_src.size(); i++)
    {
        CloudPoint pt_src_obs = m_stCloudPoints_src.at(i);
        // do a knn search
        Eigen::Vector3d pt_src_obs_vector = {pt_src_obs.x, pt_src_obs.y, 0};
        Eigen::Vector3d query_pt = pt_src_obs_vector;  // unify coordinate systems
        size_t num_results = 1;
        std::vector<size_t> ret_index(num_results);
        std::vector<double> out_dist_sqr(num_results);
        num_results = m_pKDtree_target->knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

        // check whether distance is too big
        size_t target_index = ret_index[0];
        double dist_sqr     = out_dist_sqr[0];
        if(dist_sqr > 5+1E-6)
        {
            m_vAssociation.emplace_back(std::make_pair(-1,-1));
            continue;
        }
        m_vAssociation.emplace_back(std::make_pair(target_index,dist_sqr));
    }
    */
}
