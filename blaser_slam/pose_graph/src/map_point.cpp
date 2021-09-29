//
// Created by dcheng on 10/22/19.
//

#include "map_point.h"

long unsigned int MapPoint::next_id_ = 0; // initialize the next_id_ to be 0.

void MapPoint::reprojWorldPos()
{
  //fixme something wrong. Looks like doubled. check ref KF pos.
    world_pos_ = ref_KF_->R_w_i * ref_KF_pos_ + ref_KF_->T_w_i;
}

MapPoint::MapPoint(PoseGraph* _pose_graph, Vector3d &_pos, std::vector<int> &_kf_indices, std::vector<float> &_u_in_image,
                   std::vector<float> &_v_in_image, std::vector<Vector3d> &_pos_in_image)
: pose_graph_(_pose_graph)
, world_pos_(_pos)
, weight_(_kf_indices.size())
, is_active_(true)
, ref_obs_ind_(-1)
{
    //! 0. Load up containers
    int n_observations = _kf_indices.size();
    kf_obs_ = std::vector<Observation>(n_observations);
    for (int i = 0; i < n_observations; i++)
    {
        kf_obs_[i].kf_indice = _kf_indices[i];
        kf_obs_[i].uv.x = _u_in_image[i];
        kf_obs_[i].uv.y = _v_in_image[i];
    }

    //! 1. Trace back to images and compute BRIEFs

    std::vector<BRIEF::bitset> brief_of_point(n_observations);
    for (int i = 0; i < n_observations; i++)
    {
        KeyFrame* kf =  pose_graph_->getKeyFrame(_kf_indices[i]);
        size_t j = 0;
        for (j = 0; j < kf->point_2d_uv.size(); j++)
        {
            if (kf->point_2d_uv[j].x == _u_in_image[i] && kf->point_2d_uv[j].y == _v_in_image[i])
            {
                brief_of_point[i] = kf->window_brief_descriptors[j];
                break;
            }
        }
        ROS_ASSERT(j != kf->point_2d_uv.size() && "2D point not found in MapPoint!");

        // used when point id is available for each point in each image.
        //brief_of_point[i] = pose_graph_->getKeyFrame(_kf_indices[i])->brief_descriptors[_id_in_image[i]];
    }

    //! 2. Find the best BRIEF that has the smallest Hamming distance to others
    MatrixXi hamming_dist(n_observations, n_observations);
    for (int i = 0; i < n_observations - 1; i++){
        for (int j = i + 1; j < n_observations; j++)
        {
            int dist = BRIEF::distance(brief_of_point[i], brief_of_point[j]);
            hamming_dist(i, j) = hamming_dist(j, i) = dist;
        }
        hamming_dist(i, i) = 0;
    }
    int min_dist_idx, min_dist;
    //std::cout << hamming_dist.rows() << ", " << hamming_dist.cols() << std::endl;
    min_dist = hamming_dist.colwise().sum().minCoeff(&min_dist_idx);
    ref_KF_ = pose_graph_->getKeyFrame(_kf_indices[min_dist_idx]);
    ref_KF_ind_ = _kf_indices[min_dist_idx];
    ref_obs_ind_ = min_dist_idx;
    ref_kf_obs_ = kf_obs_[ref_obs_ind_];
    ref_KF_pos_ = ref_KF_->R_w_i.transpose() * (world_pos_ - ref_KF_->T_w_i);
    ROS_DEBUG("Min dist of point is %d, ref KF index is %d", min_dist, min_dist_idx);

    //! 3. Compute feature descriptor of point in ref_KF
    //todo use BRIEF for matching for now, design a super feature later
    //  SuperFeature: Use transformation to image patch (from TSDF's normal and viewing angle) to fully compensate
    //  for scale, viewing angle, and rotation.
    //  Discretize the transformation space (like 12 degrees in ORB) and pre-compute pixel coordinates
    //  What's the reference KF in that case?
    descriptor_ = brief_of_point[min_dist_idx];

    //! 4. Load up the attributes
    id_ = next_id_++;
    min_range_ = 0.0;
    max_range_ = std::numeric_limits<float>::max();
    Vector3d norm_vec_sum = Vector3d::Zero();

    for (int i = 0; i < n_observations; i++)
    {
        float range = _pos_in_image[i].norm();
        min_range_ = std::min(min_range_, range);
        max_range_ = std::max(max_range_, range);
        //Vector3d rel_pos = _rel_R[i] * world_pos_ + _rel_t[i];
        norm_vec_sum += _pos_in_image[i] / range;
    }
    normal_vector_ = norm_vec_sum / norm_vec_sum.norm();
}

void MapPoint::mergeObservation(MapPoint &_old_point)
{
    ROS_ASSERT(is_active_ == true);
    world_pos_ = (world_pos_ * weight_ + _old_point.world_pos_ * _old_point.weight_) / (weight_ + _old_point.weight_);
    weight_ += _old_point.weight_;

    // todo decide ref_KF based on viewing angle (verticle) and distance, and update descriptor
    //   for now just keep the old one (by doing nothing)

    Vector3d norm_vec_sum = normal_vector_ * weight_ + _old_point.normal_vector_ * _old_point.weight_;
    normal_vector_ = norm_vec_sum / norm_vec_sum.norm();

    min_range_ = std::min(min_range_, _old_point.min_range_);
    max_range_ = std::max(max_range_, _old_point.max_range_);
}

MapPoint::MapPoint(Vector3d &_pos)
: world_pos_(_pos)
{

}
