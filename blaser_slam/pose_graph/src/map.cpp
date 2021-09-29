//
// Created by dcheng on 10/22/19.
//
#include "map.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>


LandmarkMap::LandmarkMap()
    : skip_cnt_(0)
//, kdtree_inactive_(v_inactive_points_, false)
{
  PUB_CLOUD_IN_REGISTRATION = true;
  initROSMsg();
}

void LandmarkMap::addMapPoints(std::vector<MapPoint *> &_vmp)
{
  const bool add_inactive = true;
  m_mp_.lock();
  v_frame_size_.push_back(_vmp.size());
  v_active_points_.insert(v_active_points_.end(), _vmp.begin(), _vmp.end());

  if (v_frame_size_.size() > ACT_KF_SIZE)
  {
    if (add_inactive)
    {
      // add active points to inactive points, update kd-tree
      size_t inact_add_start = v_inactive_points_.size();
      v_inactive_points_.insert(v_inactive_points_.end(),
                                v_active_points_.begin(),
                                v_active_points_.begin() +
                                v_frame_size_.front());
      //kdtree_inactive_.addPoints(v_inactive_points_, inact_add_start, v_inactive_points_.size());
    }

    // purge old frame
    v_active_points_.erase(v_active_points_.begin(),
                           v_active_points_.begin() + v_frame_size_.front());
    v_frame_size_.pop_front();
  }
  m_mp_.unlock();
  //pubActiveInactiveCloud();
}

void LandmarkMap::eraseMapPoint(int idx)
{
  v_inactive_points_.erase(v_inactive_points_.begin() + idx);
  //kdtree_inactive_.removePoint(idx);
}

bool LandmarkMap::registerActive(Matrix3f &R_ai, Vector3f &t_ai,
                                 size_t &inact_kf_ind, size_t &act_kf_ind)
{
  //return false;
  if (skip_cnt_ > 0)
  {
    skip_cnt_--;
    return false;
  }
  if (v_inactive_points_.empty())
    return false;

  //! 1. Bring up the inactive map points in the vicinity of the current active map (center for now)
  /*
  Vector3d act_center = Vector3d::Zero();
  for (auto &amp : v_active_points_)
      act_center += amp->world_pos_;
  act_center /= v_active_points_.size();

  double max_dist_to_center = 0.0;
  for (auto &amp : v_active_points_)
      max_dist_to_center = std::max(max_dist_to_center, (amp->world_pos_ - act_center).norm());

  std::vector<int> k_indices;
  std::vector<double> k_sqr_dist;
  int n_found = kdtree_inactive_.radiusSearch(act_center, 1.2 * max_dist_to_center, k_indices, k_sqr_dist);

  cout << "inactive map points found near active map points: " << n_found << endl
       << "total number of inactive map points: " << v_inactive_points_.size() << endl;

  if (n_found < 50) // if no inactive points in the vicinity of active points
      return false;

  std::vector<MapPoint *> v_inact_mp_roi_(n_found);
  for (size_t i = 0; i < n_found; i++)
      v_inact_mp_roi_[i] = v_inactive_points_[k_indices[i]];

  // publish inactive point cloud in the vicinity of the active map
  if (PUB_CLOUD_IN_REGISTRATION)
  {
      sensor_msgs::PointCloud inact_roi_pc_msg;
      inact_roi_pc_msg.header.stamp = ros::Time(v_active_points_.back()->ref_KF_->time_stamp);
      inact_roi_pc_msg.header.seq   = v_active_points_.back()->ref_KF_ind_;
      inact_roi_pc_msg.header.frame_id = "world";

      for (auto &inact_mp_roi : v_inact_mp_roi_)
      {
          geometry_msgs::Point32 p_inact;
          p_inact.x = inact_mp_roi->world_pos_[0];
          p_inact.y = inact_mp_roi->world_pos_[1];
          p_inact.z = inact_mp_roi->world_pos_[2];

          inact_roi_pc_msg.points.push_back(p_inact);
      }

      pub_inact_match_cloud.publish(inact_roi_pc_msg);
  }
  */

  pubActiveInactiveCloud();
  //! 2. Establish feature correspondences between active and inactive map points (brute force for now)
  //!    and build pcl point clouds
  std::vector<int> corresp(v_active_points_.size(), -1);
  static const int TH_BRIEF_DIST = 65;
  static const double TH_SEARCH_RADIUS = 0.008; // 0.8 cm
  int DEBUG_LOOP_CLOSURE = 0;
  size_t n_pairs = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr act_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZ>),
      inact_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // indices for each point in pcl point cloud in original vector

  std::vector<std::pair<size_t, size_t>> pc_corresp_idx;

  // build kd-tree
  m_mp_.lock();
  nanoflann::KDTreeMapPoint kdtree_inact(v_inactive_points_);

  // establish correspondences
  for (size_t i = 0; i < v_active_points_.size(); i++)
  {
    // find inact points near this active point
    std::vector<int> k_indices;
    std::vector<double> k_sqr_dist;
    int n_found = kdtree_inact.radiusSearch(v_active_points_[i]->world_pos_,
                                            pow(TH_SEARCH_RADIUS, 2),
                                            k_indices, k_sqr_dist);

    int min_dist = std::numeric_limits<int>::max();
    for (int j = 0; j < n_found; j++)
    {
      int dist = BRIEF::distance(v_active_points_[i]->descriptor_,
                                 v_inactive_points_[k_indices[j]]->descriptor_);
      if (dist < min_dist && dist < TH_BRIEF_DIST)
      {
        min_dist = dist;
        corresp[i] = k_indices[j];
      }
    }
    if (corresp[i] != -1)
    {
      bool cur_match_valid_manual = true; // used for debug, manually decide whether to add match
      if (DEBUG_LOOP_CLOSURE)
      {
        cur_match_valid_manual = false;
        // show the corresponding correspondence
        geometry_msgs::Point p_act, p_inact;
        p_act.x = v_active_points_[i]->world_pos_[0];
        p_act.y = v_active_points_[i]->world_pos_[1];
        p_act.z = v_active_points_[i]->world_pos_[2];
        p_inact.x = v_inactive_points_[corresp[i]]->world_pos_[0];
        p_inact.y = v_inactive_points_[corresp[i]]->world_pos_[1];
        p_inact.z = v_inactive_points_[corresp[i]]->world_pos_[2];

        corresp_line_debug_.header.stamp = ros::Time::now();
        corresp_line_debug_.points.clear();
        corresp_line_debug_.points.push_back(p_act);
        corresp_line_debug_.points.push_back(p_inact);

        pub_corresp_debug_marker.publish(corresp_line_debug_);

        // show image pair
        cv::Mat act_im, inact_im, act_inact_im;
        v_active_points_[i]->ref_KF_->loadImage(act_im);
        v_inactive_points_[corresp[i]]->ref_KF_->loadImage(inact_im);
        cv::circle(act_im, v_active_points_[i]->ref_kf_obs_.uv, 10,
                   cv::Scalar(0, 0, 255), 2);

        for (auto &inact_p : v_inactive_points_)
          for (auto &obs : inact_p->kf_obs_)
            if (obs.kf_indice == v_inactive_points_[corresp[i]]->ref_KF_ind_)
              cv::circle(inact_im, obs.uv, 10, cv::Scalar(0, 255, 0));

        cv::circle(inact_im, v_inactive_points_[corresp[i]]->ref_kf_obs_.uv, 10,
                   cv::Scalar(0, 0, 255), 2);

        cv::hconcat(act_im, inact_im, act_inact_im);
        cv::imshow("loop closure debug", act_inact_im);
        cv::waitKey(5);
        cout << "show image" << endl;
      }
      if (cur_match_valid_manual)
      {
        n_pairs++;
        // build pcl clouds
        pcl::PointXYZ act_mp(v_active_points_[i]->world_pos_[0],
                             v_active_points_[i]->world_pos_[1],
                             v_active_points_[i]->world_pos_[2]);
        pcl::PointXYZ inact_mp(v_inactive_points_[corresp[i]]->world_pos_[0],
                               v_inactive_points_[corresp[i]]->world_pos_[1],
                               v_inactive_points_[corresp[i]]->world_pos_[2]);
        act_pc_ptr->push_back(act_mp);
        inact_pc_ptr->push_back(inact_mp);
        pc_corresp_idx.emplace_back(i, corresp[i]);
      } else
      {
        corresp[i] = -1;
      }
    }
  }

  // set up pcl correspondences. Since we stack correspondences into point clouds, indices should be the same.
  pcl::CorrespondencesPtr pcl_corresp(new pcl::Correspondences);
  pcl_corresp->resize(n_pairs);
  for (size_t i = 0; i < n_pairs; i++)
    (*pcl_corresp)[i].index_query = (*pcl_corresp)[i].index_match = i;

  ROS_INFO("Correspondences %d pairs", n_pairs);
  if (n_pairs < TH_NUM_INLIER)
  {
    m_mp_.unlock();
    return false;
  }

  // publish correspondences markers
  if (PUB_CLOUD_IN_REGISTRATION)
  {
    //corresp_line_list_.header.stamp = ros::Time::now();//ros::Time(v_active_points_.back()->ref_KF_->time_stamp);
    //corresp_line_list_.header.seq   = v_active_points_.back()->ref_KF_ind_;
    corresp_line_list_.color.g = 0;
    corresp_line_list_.color.b = 1;
    corresp_line_list_.points.clear();
    geometry_msgs::Point p_act, p_inact;
    for (size_t i = 0; i < corresp.size(); i++)
    {
      if (corresp[i] == -1)
        continue;
      p_act.x = v_active_points_[i]->world_pos_[0];
      p_act.y = v_active_points_[i]->world_pos_[1];
      p_act.z = v_active_points_[i]->world_pos_[2];

      p_inact.x = v_inactive_points_[corresp[i]]->world_pos_[0];
      p_inact.y = v_inactive_points_[corresp[i]]->world_pos_[1];
      p_inact.z = v_inactive_points_[corresp[i]]->world_pos_[2];

      corresp_line_list_.points.push_back(p_act);
      corresp_line_list_.points.push_back(p_inact);
    }
    pub_corresp_marker.publish(corresp_line_list_);
  }

  //! 3. 3D-3D registration using RANSAC (PCL), try to register the inactive map to the active map (T_inact_act)

  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> ransac_rejector;
  // set ransac parameters
  ransac_rejector.setInlierThreshold(TH_RANSAC_DIST); // in meters
  ransac_rejector.setRefineModel(false); // refine parameters on the fly
  ransac_rejector.setMaximumIterations(1000); // default is 1000
  ransac_rejector.setInputSource(inact_pc_ptr);
  ransac_rejector.setInputTarget(act_pc_ptr);
  ransac_rejector.setInputCorrespondences(pcl_corresp);

  pcl::Correspondences pcl_corresp_inliers;
  std::vector<int> rejected_indices, accepted_indices;
  ransac_rejector.getCorrespondences(pcl_corresp_inliers);
  ransac_rejector.getRejectedQueryIndices(pcl_corresp_inliers,
                                          rejected_indices);
  accepted_indices.reserve(pcl_corresp_inliers.size());
  for (int i = 0, j = 0; i < pcl_corresp->size(); i++)
  {
    if (i != rejected_indices[j] || j >= rejected_indices.size())
      accepted_indices.push_back(i);
    else
      j++;
  }

  // catch ransac failure by inlier number
  cout << "RANSAC inlier number: " << pcl_corresp_inliers.size() << endl;
  if (pcl_corresp_inliers.size() < TH_NUM_INLIER)
  {
    ROS_INFO("RANSAC not enough inliers, map matching abandoned!");
    m_mp_.unlock();
    return false;
  }

  // publish RANSAC inlier correspondences before alignment
  if (PUB_CLOUD_IN_REGISTRATION)
  {
    corresp_line_list_.color.g = 1;
    corresp_line_list_.color.b = 1;
    corresp_line_list_.points.clear();
    geometry_msgs::Point p_act, p_inact;
    for (auto &pcl_corresp_inlier : pcl_corresp_inliers)
    {
      p_act.x = act_pc_ptr->points[pcl_corresp_inlier.index_match].x;
      p_act.y = act_pc_ptr->points[pcl_corresp_inlier.index_match].y;
      p_act.z = act_pc_ptr->points[pcl_corresp_inlier.index_match].z;

      p_inact.x = inact_pc_ptr->points[pcl_corresp_inlier.index_query].x;
      p_inact.y = inact_pc_ptr->points[pcl_corresp_inlier.index_query].y;
      p_inact.z = inact_pc_ptr->points[pcl_corresp_inlier.index_query].z;

      corresp_line_list_.points.push_back(p_act);
      corresp_line_list_.points.push_back(p_inact);
    }
    pub_corresp_debug_marker.publish(corresp_line_list_);
  }

  // compute centroid of two landmarks maps
  Vector3f centroid_act(0, 0, 0), centroid_inact(0, 0, 0);
  for (auto &pcl_corresp_inlier : pcl_corresp_inliers)
  {
    centroid_act += Vector3f(
        act_pc_ptr->points[pcl_corresp_inlier.index_match].x,
        act_pc_ptr->points[pcl_corresp_inlier.index_match].y,
        act_pc_ptr->points[pcl_corresp_inlier.index_match].z);
    centroid_inact += Vector3f(
        inact_pc_ptr->points[pcl_corresp_inlier.index_query].x,
        inact_pc_ptr->points[pcl_corresp_inlier.index_query].y,
        inact_pc_ptr->points[pcl_corresp_inlier.index_query].z);
  }
  centroid_act /= pcl_corresp_inliers.size();
  centroid_inact /= pcl_corresp_inliers.size();

  // solve SE(3) with inlier correspondences using SVD
  Matrix4f trans_ai_6dof; // transformation from inactive to active.
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> se3_svd_est;
  se3_svd_est.estimateRigidTransformation(*inact_pc_ptr, *act_pc_ptr,
                                          pcl_corresp_inliers, trans_ai_6dof);
  // convert the 6DoF transformation into a 4DoF one (yaw and translation)
  R_ai = trans_ai_6dof.block<3, 3>(0, 0);
  R_ai = Utility::ypr2R(Vector3f(Utility::R2ypr(R_ai).x(), 0, 0));
  t_ai = centroid_act - R_ai * centroid_inact;
  Matrix4f trans_ai_4dof;
  trans_ai_4dof << R_ai, t_ai, 0, 0, 0, 1.;

  //! 4. Decide if registration is successful: enough inliers, final residual is small, and Hessian's eigenvalues
  //!    are small (idea from ElasticFusion).
  static const double TH_REGISTRATION_RESIDUAL = 1.0;
  static const double TH_MERGE_PAIR = pow(0.005,
                                          2); // squared distance threshold

  // compute residual
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*inact_pc_ptr, *cloud_source_trans, trans_ai_4dof);
  double residual = 0.0;
  // std::vector<std::pair<size_t, size_t> > corresp_merge;
  for (auto &pcl_corresp_inlier : pcl_corresp_inliers)
  {
    pcl::PointXYZ src_trans = (*cloud_source_trans)[pcl_corresp_inlier.index_query];
    pcl::PointXYZ target = (*act_pc_ptr)[pcl_corresp_inlier.index_match];
    double dist_sqr = pow(src_trans.x - target.x, 2)
                      + pow(src_trans.y - target.y, 2)
                      + pow(src_trans.z - target.z, 2);
    residual += dist_sqr;
    // todo can add normal vector to make this point-to-plane match.
  }
  residual /= pcl_corresp_inliers.size();

  // todo add Hessian = (J' J)^-1
  bool is_regist_success = pcl_corresp_inliers.size() > TH_NUM_INLIER
                           && residual < TH_REGISTRATION_RESIDUAL;

  // publish RANSAC inlier correspondences after alignment
  bool DEBUG_INLIER_CORRESP_IMG = false;
  if (PUB_CLOUD_IN_REGISTRATION)
  {
    sensor_msgs::PointCloud inact_align_pc_msg;
    inact_align_pc_msg.header.stamp = ros::Time(
        v_active_points_.back()->ref_KF_->time_stamp);
    inact_align_pc_msg.header.seq = v_active_points_.back()->ref_KF_ind_;
    inact_align_pc_msg.header.frame_id = "world";

    corresp_aft_align_list_.points.clear();

    geometry_msgs::Point p_act, p_inact;
    geometry_msgs::Point32 p32_inact;
    for (auto &pcl_corresp_inlier : pcl_corresp_inliers)
    {
      p_act.x = act_pc_ptr->points[pcl_corresp_inlier.index_match].x;
      p_act.y = act_pc_ptr->points[pcl_corresp_inlier.index_match].y;
      p_act.z = act_pc_ptr->points[pcl_corresp_inlier.index_match].z;

      p_inact.x = (*cloud_source_trans)[pcl_corresp_inlier.index_query].x;
      p_inact.y = (*cloud_source_trans)[pcl_corresp_inlier.index_query].y;
      p_inact.z = (*cloud_source_trans)[pcl_corresp_inlier.index_query].z;

      p32_inact.x = (*cloud_source_trans)[pcl_corresp_inlier.index_query].x;
      p32_inact.y = (*cloud_source_trans)[pcl_corresp_inlier.index_query].y;
      p32_inact.z = (*cloud_source_trans)[pcl_corresp_inlier.index_query].z;

      inact_align_pc_msg.points.push_back(p32_inact);
      corresp_aft_align_list_.points.push_back(p_act);
      corresp_aft_align_list_.points.push_back(p_inact);
    }
    pub_corresp_aft_align_marker.publish(corresp_aft_align_list_);
    pub_inact_match_cloud.publish(inact_align_pc_msg);

    // debug image for validating correspondences
    if (DEBUG_INLIER_CORRESP_IMG)
    {
      for (auto &idx_pair : pc_corresp_idx)
      {
        cv::Mat act_im, inact_im, act_inact_im;
        v_active_points_[idx_pair.first]->ref_KF_->loadImage(act_im);
        v_inactive_points_[idx_pair.second]->ref_KF_->loadImage(inact_im);
        cv::circle(act_im, v_active_points_[idx_pair.first]->ref_kf_obs_.uv, 10,
                   cv::Scalar(0, 0, 255), 2);

        for (auto &inact_p : v_inactive_points_)
          for (auto &obs : inact_p->kf_obs_)
            if (obs.kf_indice ==
                v_inactive_points_[idx_pair.second]->ref_KF_ind_)
              cv::circle(inact_im, obs.uv, 10, cv::Scalar(0, 255, 0));

        cv::circle(inact_im,
                   v_inactive_points_[idx_pair.second]->ref_kf_obs_.uv, 10,
                   cv::Scalar(0, 0, 255), 2);

        cv::hconcat(act_im, inact_im, act_inact_im);
        cv::imshow("loop closure debug", act_inact_im);
        cv::waitKey(5);
        cout << "show image" << endl;
      }
    }
  }

  //! 5. If successful, send transform out
  const bool MERGE_CORRESP = true;
  if (is_regist_success)
  {
    ROS_INFO("Registration successfull!");

    // transform active map todo delete this when doing pose graph
    {
      Matrix3f R_ia_f = R_ai.transpose();
      Vector3f t_ia_f = -R_ia_f * t_ai;
      Matrix3d R_ia = R_ia_f.cast<double>();
      Vector3d t_ia = t_ia_f.cast<double>();

      for (auto &act_mp : v_active_points_)
      {
        act_mp->world_pos_ = R_ia * act_mp->world_pos_ + t_ia;

        // pose in ref. KF. should not change, since it is direct and local observation.
        //act_mp->ref_KF_pos_ = R_ai * act_mp->world_pos_ + t_ai;
      }
      pubActiveInactiveCloud();
    }

    // obtain middle kf of active landmark maps
    /*
    int middle_mp_ind = 0;
    for (size_t i = 0; i < ACT_KF_SIZE / 2; i++)
        middle_mp_ind += v_frame_size_[i];
    act_kf_ind = v_active_points_[middle_mp_ind]->ref_KF_ind_;
    */
    int ref_kf_ind_sum = 0;
    for (auto act_mp : v_active_points_)
      ref_kf_ind_sum += act_mp->ref_KF_ind_;
    act_kf_ind = ref_kf_ind_sum / v_active_points_.size();

    //act_kf_ind = v_active_kf_[ACT_KF_SIZE / 2];

    // find the middle kf of the matched part of the inactive map
    int kf_ind_sum = 0;
    for (int accepted_idx : accepted_indices)
    {
      kf_ind_sum += v_inactive_points_[pc_corresp_idx[accepted_idx].second]->ref_KF_ind_;
    }
    inact_kf_ind = kf_ind_sum / pcl_corresp_inliers.size();

    // merge matched landmarks: update active observation, delete inactive landmark
    // sort inact indices to delete in descending order, so deleting elements won't change the indices of the
    // later ones
    //

    if (MERGE_CORRESP)
    {
      const double MERGE_DIST_SQR = pow(0.002, 2), MERGE_BRIEF_HD = 60;
      std::vector<std::pair<MapPoint *, size_t> > corresp_merge;

      // find correspondences after alignment of all active map points
      for (auto &act_mp : v_active_points_)
      {
        std::vector<int> k_indices;
        std::vector<double> k_sqr_dist;
        int n_found = kdtree_inact.radiusSearch(act_mp->world_pos_,
                                                MERGE_DIST_SQR,
                                                k_indices, k_sqr_dist);

        int min_dist = std::numeric_limits<int>::max(), min_idx;
        for (int j = 0; j < n_found; j++)
        {
          int dist = BRIEF::distance(act_mp->descriptor_,
                                     v_inactive_points_[k_indices[j]]->descriptor_);
          if (dist < min_dist && dist < MERGE_BRIEF_HD)
          {
            min_dist = dist;
            min_idx = k_indices[j];
          }
        }
        if (min_dist != std::numeric_limits<int>::max())
          corresp_merge.emplace_back(act_mp, min_idx);
      }

      if (PUB_CLOUD_IN_REGISTRATION)
      {
        merge_pair_line_list_.points.clear();
        geometry_msgs::Point p_act, p_inact;
        for (auto &pair : corresp_merge)
        {
          p_act.x = pair.first->world_pos_[0];
          p_act.y = pair.first->world_pos_[1];
          p_act.z = pair.first->world_pos_[2];

          p_inact.x = v_inactive_points_[pair.second]->world_pos_[0];
          p_inact.y = v_inactive_points_[pair.second]->world_pos_[1];
          p_inact.z = v_inactive_points_[pair.second]->world_pos_[2];

          merge_pair_line_list_.points.push_back(p_act);
          merge_pair_line_list_.points.push_back(p_inact);
        }
        pub_merge_pair.publish(merge_pair_line_list_);
      }

      std::sort(corresp_merge.begin(), corresp_merge.end(),
                [](std::pair<MapPoint *, size_t> const &a,
                   std::pair<MapPoint *, size_t> const &b) -> bool
                { return a.second > b.second; });

      for (auto &corresp : corresp_merge)
      {
        corresp.first->mergeObservation(*v_inactive_points_[corresp.second]);
        eraseMapPoint(corresp.second);
      }
      pubActiveInactiveCloud();
      m_mp_.unlock();
      skip_cnt_ = ACT_KF_SIZE;
      return true;
    }
  } else
  {
    m_mp_.unlock();
    return false;
  }
}

LandmarkMap::~LandmarkMap()
{
  for (auto &p_active_point : v_active_points_)
    delete p_active_point;

  for (auto &p_inactive_point : v_inactive_points_)
    delete p_inactive_point;
}

void LandmarkMap::pubActiveInactiveCloud()
{
  sensor_msgs::PointCloud act_pc_msg, inact_pc_msg;
  act_pc_msg.header.frame_id = inact_pc_msg.header.frame_id = "world";
  act_pc_msg.header.stamp = inact_pc_msg.header.stamp = ros::Time(
      v_active_points_.back()->ref_KF_->time_stamp);
  act_pc_msg.header.seq = inact_pc_msg.header.seq = v_active_points_.back()->ref_KF_ind_;

  //m_mp_.lock();
  for (auto &act_mp : v_active_points_)
  {
    geometry_msgs::Point32 act_p;
    act_p.x = act_mp->world_pos_[0];
    act_p.y = act_mp->world_pos_[1];
    act_p.z = act_mp->world_pos_[2];

    act_pc_msg.points.push_back(act_p);
  }

  for (auto &inact_mp : v_inactive_points_)
  {
    geometry_msgs::Point32 inact_p;
    inact_p.x = inact_mp->world_pos_[0];
    inact_p.y = inact_mp->world_pos_[1];
    inact_p.z = inact_mp->world_pos_[2];

    inact_pc_msg.points.push_back(inact_p);
  }

  //m_mp_.unlock();
  pub_act_cloud.publish(act_pc_msg);
  pub_inact_cloud.publish(inact_pc_msg);
}

void LandmarkMap::initROSMsg()
{
  corresp_line_list_.header.frame_id = "world";
  //corresp_line_list_.action = visualization_msgs::Marker::ADD;
  corresp_line_list_.pose.orientation.w = 1.0;
  corresp_line_list_.id = 0;
  corresp_line_list_.type = visualization_msgs::Marker::LINE_LIST;
  corresp_line_list_.scale.x = 0.0003; // meters
  corresp_line_list_.color.a = 1.0;
  corresp_line_list_.color.b = 1.0;

  corresp_aft_align_list_.header.frame_id = "world";
  corresp_aft_align_list_.action = visualization_msgs::Marker::ADD;
  corresp_aft_align_list_.pose.orientation.w = 1.0;
  corresp_aft_align_list_.id = 0;
  corresp_aft_align_list_.type = visualization_msgs::Marker::LINE_LIST;
  corresp_aft_align_list_.scale.x = 0.0003; // meters
  corresp_aft_align_list_.color.a = 1.0;
  corresp_aft_align_list_.color.r = 1.0;
  corresp_aft_align_list_.color.g = 1.0;

  corresp_line_debug_.header.frame_id = "world";
  corresp_line_debug_.action = visualization_msgs::Marker::ADD;
  corresp_line_debug_.pose.orientation.w = 1.0;
  corresp_line_debug_.id = 0;
  corresp_line_debug_.type = visualization_msgs::Marker::LINE_STRIP;
  corresp_line_debug_.scale.x = 0.0007; // meters
  corresp_line_debug_.color.a = 1.0;
  corresp_line_debug_.color.g = 1.0;

  merge_pair_line_list_.header.frame_id = "world";
  merge_pair_line_list_.action = visualization_msgs::Marker::ADD;
  merge_pair_line_list_.pose.orientation.w = 1.0;
  merge_pair_line_list_.id = 0;
  merge_pair_line_list_.type = visualization_msgs::Marker::LINE_LIST;
  merge_pair_line_list_.scale.x = 0.0003; // meters
  merge_pair_line_list_.color.a = 1.0;
  merge_pair_line_list_.color.b = 0.8; // purple
  merge_pair_line_list_.color.r = 0.8;
}

void LandmarkMap::updateMapPoints()
{
  m_mp_.lock();
  // update position of active and inactive map points
  for (auto pmp : v_active_points_)
    pmp->reprojWorldPos();

  for (auto pmp : v_inactive_points_)
    pmp->reprojWorldPos();

  // update kd-tree
  // following line no longer needed, since now use non-dynamic
  //new(&kdtree_inactive_) nanoflann::KDTreeDynamicMapPoint(v_inactive_points_, false);

  // todo merge close and similar map points. Don't need to do that for now

  m_mp_.unlock();

  pubActiveInactiveCloud();
}

void LandmarkMap::registerPub(ros::NodeHandle &nh)
{
  pub_act_cloud = nh.advertise<sensor_msgs::PointCloud>("act_cloud", 10);
  pub_inact_cloud = nh.advertise<sensor_msgs::PointCloud>("inact_cloud", 10);
  pub_inact_match_cloud = nh.advertise<sensor_msgs::PointCloud>(
      "inact_match_cloud", 10);
  pub_corresp_marker = nh.advertise<visualization_msgs::Marker>(
      "corresp_marker", 10);
  pub_corresp_aft_align_marker = nh.advertise<visualization_msgs::Marker>(
      "corresp_marker_aft_align", 10);
  pub_corresp_debug_marker = nh.advertise<visualization_msgs::Marker>(
      "corresp_marker_debug", 10);
  pub_merge_pair = nh.advertise<visualization_msgs::Marker>("merge_pair", 10);
}
