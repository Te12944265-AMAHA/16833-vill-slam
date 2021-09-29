//
// Created by dcheng on 11/27/19.
//

#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>

int main(int argc, char** argv)
{
    //! 1. Generate random point coordinates
    const size_t N_PTS = 10;
    Eigen::MatrixXd input_coords = Eigen::MatrixXd::Random(3, N_PTS);
    Eigen::Matrix3d R;
    R << 0, -1, 0,
         1, 0, 0,
         0, 0, 1;
    Eigen::MatrixXd output_coords = R * input_coords;

    //! 2. Set up PCL point clouds and correspondences
    pcl::PointCloud<pcl::PointXYZ>::Ptr act_pc_ptr  (new pcl::PointCloud<pcl::PointXYZ>),
                                        inact_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CorrespondencesPtr pcl_corresp(new pcl::Correspondences);
    pcl_corresp->resize(N_PTS);
    for (size_t i = 0; i < N_PTS; i++)
    {
        pcl::PointXYZ act_mp  (input_coords(0, i) , input_coords(1, i) , input_coords(2, i));
        pcl::PointXYZ inact_mp(output_coords(0, i), output_coords(1, i), output_coords(2, i));
        act_pc_ptr->push_back(act_mp);
        inact_pc_ptr->push_back(inact_mp);

        (*pcl_corresp)[i].index_query = i;
        (*pcl_corresp)[i].index_match = i;
    }

    //! 3. Apply ransac
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> ransac_rejector;
    ransac_rejector.setInlierThreshold(0.05);
    ransac_rejector.setRefineModel(true);
    ransac_rejector.setMaximumIterations(100);
    ransac_rejector.setInputSource(inact_pc_ptr);
    ransac_rejector.setInputTarget(act_pc_ptr);
    ransac_rejector.setInputCorrespondences(pcl_corresp);

    pcl::Correspondences pcl_corresp_inliers;
    ransac_rejector.getCorrespondences(pcl_corresp_inliers);

    //! 4. Obtain transformation
    Eigen::Matrix4f T;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> se3_svd_est;
    se3_svd_est.estimateRigidTransformation(*inact_pc_ptr, *act_pc_ptr, pcl_corresp_inliers, T);
    std::cout << "Solved transformation:\n" << T << std::endl;
}