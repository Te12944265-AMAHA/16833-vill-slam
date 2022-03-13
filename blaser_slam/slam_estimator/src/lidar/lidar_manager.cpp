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
#include "../utility/geometry_utils.h"
#include <pcl/registration/icp.h>
#include "../utility/tic_toc.h"

#define LIDAR_ITER 10
#define DT_CONVERGE_THRESH 1e-5
#define DQ_CONVERGE_THRESH 1e-5

LidarManager::LidarManager()
{
    LidarFactor::sqrt_info = 1e1;
}

void LidarManager::addLidarDataFrame(LidarDataFrameConstPtr frame, double stamp)
{
    lidar_window_.insert(std::pair<double, LidarDataFrameConstPtr>(stamp, frame));
}

void LidarManager::discardObsoleteReadings(double time)
{
    auto it = lidar_window_.begin();
    while (it != lidar_window_.end() && it->first < time)
    {
        it = lidar_window_.erase(it);
    }
    ROS_DEBUG("remaining frames in lidar window: %d", lidar_window_.size());
}


int LidarManager::get_lidar_frame(const double time, LidarDataFramePtr frame_out) const
{
    auto itr = lidar_window_.find(time);
    if (itr != lidar_window_.end())
    {
        if (itr->second == nullptr)
            return -2;
        else
        {
            *frame_out = *(itr->second);
            return 0;
        }
    }
    return -1;
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

// Lidar cylinder factor: currently not in use
int LidarManager::getRelativeTf(double t1, double t2, Eigen::Matrix4f &T)
{

    if (0)
    {
        ROS_WARN("Lidar factor invalid value, should only happen at beginning??");
        return -1;
    }
    LidarDataFramePtr frame1 = lidar_window_[t1];
    LidarDataFramePtr frame2 = lidar_window_[t2];
    // no interpolation if this factor is used
    return getRelativeTf(frame1->f1_p, frame2->f1_p, T);
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

// find point pairs through knn
// corrs: <point_from_target, point_from_source>
void LidarManager::associate(LidarPointCloudConstPtr source_cloud,
                             LidarPointCloudConstPtr target_cloud,
                             std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &corrs)
{

    for (int i = 0; i < source_cloud->size(); i++)
    {
        LidarPoint searchPoint = source_cloud->points[i];
        std::vector<int> pointIdxKNNSearch(K);
        std::vector<float> pointKNNSquaredDistance(K);
        // float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
        if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
        {
            if (pointKNNSquaredDistance[0] < dist_thresh)
            {
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

// find correct point correspondences
void LidarManager::align(LidarPointCloudConstPtr source_cloud,
                         LidarPointCloudConstPtr target_cloud,
                         std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &corrs,
                         Eigen::Affine3f &tf_out)
{
    double pose[SIZE_POSE] = {0, 0, 0, 0, 0, 0, 1}; // the transformation T such that dst = T * src
    double pose0[SIZE_POSE] = {0, 0, 0, 0, 0, 0, 1};
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t(pose);
    Eigen::Map<Eigen::Quaternion<double>> q(pose + 3);
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t0(pose0);
    Eigen::Map<Eigen::Quaternion<double>> q0(pose0 + 3);
    q = Eigen::Quaterniond::Identity();

    q0 = Eigen::Quaterniond::Identity();

    Eigen::Quaterniond q_prev, dq;
    Eigen::Vector3d t_prev, dt;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> tmp_corr;

    Eigen::Matrix4d T_i;
    pose2T(t.cast<double>(), q.cast<double>(), T_i);

    LidarPointCloudPtr tmp_src(new LidarPointCloud());
    *tmp_src = *source_cloud;

    resetKDtree(target_cloud);

    cout << "original pose: " << endl;
    for (int k = 0; k < SIZE_POSE; k++)
    {
        cout << pose[k] << ", ";
    }
    cout << endl;

    cout << "tf out before iters: " << endl;
    cout << tf_out.matrix() << endl;

    Eigen::Affine3d tf = Eigen::Affine3d::Identity();

    auto t_start = chrono::high_resolution_clock::now();
    // attemp to find the correct data association after a few iters
    for (int iter = 0; iter < LIDAR_ITER; iter++)
    {
        tmp_corr.clear();
        // data association
        associate(tmp_src, target_cloud, tmp_corr);
        cout << "Iter " << iter << ", #corrs = " << tmp_corr.size() << endl;

        ceres::Problem problem;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = false;
        // options.max_solver_time_in_seconds = SOLVER_TIME * 3;
        options.max_num_iterations = 100;
        ceres::Solver::Summary summary;
        // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        // ceres::LocalParameterization* q_local_parameterization =  ceres::QuaternionParameterization();

        // we're only optimizing the relative T
        problem.AddParameterBlock(pose, SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(pose0, SIZE_POSE, local_parameterization);
        // problem.AddParameterBlock(pose, 3);
        // problem.AddParameterBlock(pose, 4, q_local_parameterization);
        // problem.AddParameterBlock(pose, 3);
        // problem.AddParameterBlock(pose, 4, q_local_parameterization);

        problem.SetParameterBlockConstant(pose0);

        // find transformation that minimizes p2p icp residual
        for (int j = 0; j < tmp_corr.size(); j++)
        {
            Eigen::Vector3d p1 = tmp_corr[j].first.cast<double>();  // target
            Eigen::Vector3d p2 = tmp_corr[j].second.cast<double>(); // source
            auto lidar_factor = LidarFactor::Create(p1, p2, T_i, T_i);
            problem.AddResidualBlock(lidar_factor, NULL, pose0, pose);
        }
        /*
        for (int src_index = 0; src_index < m_vAssociation.size(); src_index++)
        {
            int target_index = m_vAssociation.at(src_index).first;
            if (target_index == -1) // no match is found for this source point
                continue;

            CloudPoint src_pt = m_stCloudPoints_src.at(src_index);
            CloudPoint target_pt = m_stCloudPoints_target.at(target_index);

            if (1) // use p2p icp
            {
                // point to point constraint
                Eigen::DiagonalMatrix<double, 2> cov(0.1, 0.1);
                Eigen::Matrix2d sqrt_information = cov;
                sqrt_information = sqrt_information.inverse().llt().matrixL();
                double weight = 1.0;
                ceres::CostFunction *cost_function = ceres::slam2d::PP2dErrorTerm::Create(src_pt.x, src_pt.y, target_pt.x, target_pt.y,
                                                                                            sqrt_information, weight);
                problem.AddResidualBlock(cost_function, loss_function, euler_array, t_array);
            }
        }
        */
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";

        cout << "pose: " << endl;
        for (int k = 0; k < SIZE_POSE; k++)
        {
            cout << pose[k] << ", ";
        }
        cout << endl;
        // transform src cloud before attempting to associate again

        tf.translation() = t;
        tf.rotate(q);
        tf_out = tf.cast<float>();
        LidarPointCloudPtr tmp_src2(new LidarPointCloud());
        pcl::transformPointCloud(*tmp_src, *tmp_src2, tf_out);
        *tmp_src = *tmp_src2;

        // check if we have converged
        calculate_delta_tf(t, q, t_prev, q_prev, dt, dq);
        t_prev = t;
        q_prev = q;
        cout << "tf out: " << endl;
        cout << tf_out.matrix() << endl;
        cout << "dt norm: " << dt.norm() << "dq norm: " << dq.norm() << endl
             << endl
             << endl
             << endl;
        if (iter > 0 && dt.norm() < DT_CONVERGE_THRESH && dq.norm() < DQ_CONVERGE_THRESH)
        {
            break;
        }
    }

    cout << "t: " << t << endl;
    cout << "q: " << q.toRotationMatrix() << endl;

    auto t_end = chrono::high_resolution_clock::now();
    double t_elapse = chrono::duration<double>(t_end - t_start).count();
    cout << "Time to align (s): " << std::setprecision(9) << t_elapse << endl;
    corrs = tmp_corr;
}

void LidarManager::align_pcl_icp(LidarPointCloudConstPtr source_cloud,
                                 LidarPointCloudConstPtr target_cloud,
                                 std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &corrs,
                                 Eigen::Affine3f &tf_out)
{
    TicToc tic_toc;
    corrs.clear();

    pcl::IterativeClosestPoint<LidarPoint, LidarPoint> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    LidarPointCloudPtr final_cloud(new LidarPointCloud());
    icp.align(*final_cloud);

    pcl::CorrespondencesPtr corr_indices = icp.correspondences_;
    for (size_t i = 0; i < corr_indices->size(); i++)
    {
        pcl::Correspondence cor = (*corr_indices)[i];
        // corrs: <point_from_target, point_from_source>
        LidarPoint p1 = (*target_cloud)[cor.index_match];
        LidarPoint p2 = (*source_cloud)[cor.index_query];
        Eigen::Vector3f p_target(p1.x, p1.y, p1.z);
        Eigen::Vector3f p_source(p2.x, p2.y, p2.z);
        corrs.push_back(make_pair(p_target, p_source));

    }

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    cout << "#corrs: " << corrs.size() << endl;

    tf_out.matrix() = icp.getFinalTransformation();

    cout << "tf out after icp: " << endl;
    cout << tf_out.matrix() << endl;
    double elapsed_ms = tic_toc.toc();
    ROS_INFO("align_pcl_icp took %f ms", elapsed_ms);
}


// df1 contains: [df1_f1  df1_prev   df1_f2];   df2 contains: [df2_f1   df2_cur  df2_f2]
// df1_prev and df2_cur are interpolated frames
// corrs are between df1_f2 and df2_f1  
// tf_1_2 are between df1_prev and df2_cur
void LidarManager::get_tf_between_data_frames(LidarDataFrameConstPtr df1,
                                              LidarDataFrameConstPtr df2,
                                              std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &corrs_df1f2_df2f1,
                                              Eigen::Affine3f &tf_1_2)
{
    ROS_DEBUG("entering get_tf_between_data_frames");
    Eigen::Affine3f T_2_3, T_prev_cur;
    if (df1->use_interpolate && df2->use_interpolate)
    {
        align_pcl_icp(df2->f1_p->get_pointcloud(), df1->f2_p->get_pointcloud(), corrs_df1f2_df2f1, T_2_3);
    }
    else if (df1->use_interpolate && !(df2->use_interpolate))
    {
        align_pcl_icp(df2->f1_p->get_pointcloud(), df1->f2_p->get_pointcloud(), corrs_df1f2_df2f1, T_2_3);
    }
    else if (!(df1->use_interpolate) && df2->use_interpolate)
    {
        align_pcl_icp(df2->f1_p->get_pointcloud(), df1->f1_p->get_pointcloud(), corrs_df1f2_df2f1, T_2_3);
    }
    else
    {
        align_pcl_icp(df2->f1_p->get_pointcloud(), df1->f1_p->get_pointcloud(), corrs_df1f2_df2f1, T_2_3);
    }
    T_prev_cur = df1->T_cur_2 * T_2_3.matrix() * df2->T_1_cur;
    tf_1_2 = T_prev_cur;
    ROS_DEBUG("done get_tf_between_data_frames");
}



int LidarManager::get_relative_tf(double t1, 
                                   double t2, 
                                   std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &corrs_df1f2_df2f1,
                                   Eigen::Matrix4d &T_prev_2,
                                   Eigen::Matrix4d &T_cur_1)
{
    ROS_DEBUG("entering get_relative_tf");
    if (lidar_window_.count(t2) == 0 || lidar_window_.count(t1) == 0 || lidar_window_[t1] == nullptr || lidar_window_[t2] == nullptr)
    {
        ROS_WARN("Lidar factor invalid value, TODO when should this happen?");
        return -1;
    }
    Eigen::Affine3f tf_1_2;
    LidarDataFramePtr df1 = lidar_window_[t1];
    LidarDataFramePtr df2 = lidar_window_[t2];
    // this is mainly to get the correspondences
    get_tf_between_data_frames(df1, df2, corrs_df1f2_df2f1, tf_1_2);
    T_prev_2 = df1->T_cur_2.cast<double>();
    T_cur_1 = invT(df2->T_1_cur.cast<double>());
    ROS_DEBUG("done get_relative_tf");
    return 0;
}