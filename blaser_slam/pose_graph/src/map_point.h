//
// Created by dcheng on 10/22/19.
//

#ifndef POSE_GRAPH_MAP_POINT_H
#define POSE_GRAPH_MAP_POINT_H

#include <eigen3/Eigen/Dense>
#include <map>
#include <ros/ros.h>
#include "pose_graph.h"

using namespace Eigen;
typedef Eigen::Matrix<bool, Dynamic, 1> VectorXb;

struct Observation
{
    int kf_indice;
    cv::Point2d uv;
};

/**
 * MapPoint class.
 * Created from high-quality points that are just marginalized from the sliding
 * window of vins_estimator.
 * Trace back to image and compute descriptor
 */
class MapPoint
{
public:
    /**
     * Receive from vins_estimator:
     * 1. Points that just got marginalized in the previous frame
     * 2. Each point's tracked keyframes (indices).
     * 3. Each point's index (or image coordinates) in each frame.
     *
     * What to do:
     * 1. Trace back to the images and get pre-computed BRIEFs.
     * 2. Compare the BRIEFs and find the best BRIEF and corresponding image (ref_KF).
     * 3. Compute the feature descriptor of the point in ref_KF.
     * 4. Load up the attributes of the point
     * @param _pos
     * @param _kf_indices
     */
    MapPoint(PoseGraph* _pose_graph, Vector3d &_pos, std::vector<int> &_kf_indices, std::vector<float> &_u_in_image,
            std::vector<float> &_v_in_image, std::vector<Vector3d> &_pos_in_image);

    /**
     * Constructor that only takes in world position. Only used for test! (like kd-tree)
     * @param _pos
     */
    MapPoint(Vector3d &_pos);

    /**
     * Reproject the point's world position w.r.t. updated ref keyframe
     * Used after pose graph optimization
     */
    void reprojWorldPos();

    /**
     * Add (merge) observation after successful 3D-3D alignment: merge the inactive map
     * Update all attributes of the point
     * @param _old_point the newly generated point to be merged.
     */
    void mergeObservation(MapPoint &_old_point);


public:
    PoseGraph* pose_graph_;

    // Unique id of each point
    long unsigned int id_;
    static long unsigned int next_id_;

    // Position in world frame
    Vector3d world_pos_;

    // Position in reference keyframe's frame
    Vector3d ref_KF_pos_;

    // Descriptor
    BRIEF::bitset descriptor_;

    // Keyframes observing the point and associated index in keyframes
    //std::map<KeyFrame*, size_t> observations_;

    // Mean viewing direction
    Vector3d normal_vector_;

    // Reference KeyFrame
    KeyFrame* ref_KF_;
    size_t ref_KF_ind_;
    int ref_obs_ind_;
    Observation ref_kf_obs_;

    // Min/max observation range for scale invariance
    float min_range_;
    float max_range_;

    // Count of visible keyframes
    int weight_;

    // Flag for active points
    bool is_active_;

    std::vector<Observation> kf_obs_;

};

#endif //POSE_GRAPH_MAP_POINT_H
