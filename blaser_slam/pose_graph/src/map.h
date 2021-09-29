//
// Created by dcheng on 10/22/19.
//

#ifndef VINS_ESTIMATOR_MAP_H
#define VINS_ESTIMATOR_MAP_H

#include "map_point.h"
#include "nanoflann_map_point.h"
#include <queue>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include "pose_graph.h"

using namespace Eigen;

class LandmarkMap
{
public:
    explicit LandmarkMap();
    ~LandmarkMap();

    /**
     * Add multiple new MapPoint. Used when a new frame of marginalized points come in.
     * Push the oldest frame into inactive.
     * @param _vmp vector of new MapPoint.
     */
    void addMapPoints(std::vector<MapPoint*> &_vmp);

    /**
     * Erase a point in map by its index. Used when points in inactive map got merged with the active.
     * @param idx
     */
    void eraseMapPoint(int idx);

    /**
     * Try to align the active map to the inactive map.
     * 1. Bring up the inactive map points in the vicinity of the current active map (todo figure out algo for this,
     *    for now just use the center point of the active map).
     * 2. Establish feature correspondences between active and inactive. Use brute-force for now, but todo use BoW.
     * 3. 3D-3D registration (RANSAC for SE(3)?)
     * 4. Decide if the registration is successful.
     * 5. If successful, merge matched points, give constraint to pose_graph. Transformation is from inactive keyframe
     *    to active keyframe (both are the middle of the windows).
     * 6. After pose_graph optimization, merge adjacent map points.
     *
     * @param R Rotation matrix between the two maps, if registration is successful.
     * @param t Translation vector between the two maps, if registration is successful.
     * @return true if registration is successfull (enough inliers, residual is small)
     */
    bool registerActive(Matrix3f &R_ai, Vector3f &t_ai, size_t &inact_kf_ind, size_t &act_kf_ind);

    void pubActiveInactiveCloud();

    void initROSMsg();

    /**
     * update map points w.r.t. their reference keyframe poses, after pose graph optimization
     * todo add map point merge.
     */
    void updateMapPoints();

    void registerPub(ros::NodeHandle& nh);

public:
    std::vector<MapPoint *> v_active_points_;
    std::vector<MapPoint *> v_inactive_points_;

    std::mutex m_mp_;

    std::deque<int> v_frame_size_;
    std::vector<KeyFrame *> v_active_kf_;

    //nanoflann::KDTreeDynamicMapPoint kdtree_inactive_;


    bool PUB_CLOUD_IN_REGISTRATION;

    int skip_cnt_;

    ros::Publisher pub_act_cloud;
    ros::Publisher pub_inact_cloud;
    ros::Publisher pub_inact_match_cloud;
    ros::Publisher pub_corresp_marker;
    ros::Publisher pub_corresp_aft_align_marker;
    ros::Publisher pub_corresp_debug_marker;
    ros::Publisher pub_merge_pair;
    visualization_msgs::Marker corresp_line_list_, corresp_line_debug_, corresp_aft_align_list_, merge_pair_line_list_;

private:
    static const uint8_t ACT_KF_SIZE = 12;
    static const size_t TH_NUM_INLIER = 15;
    constexpr static const double TH_RANSAC_DIST = 0.003;
};


#endif //VINS_ESTIMATOR_MAP_H
