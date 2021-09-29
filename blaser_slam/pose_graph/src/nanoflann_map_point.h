//
// Created by dcheng on 10/28/19.
//

#ifndef POSE_GRAPH_NANOFLANN_MAP_POINT_H
#define POSE_GRAPH_NANOFLANN_MAP_POINT_H

#include "utility/nanoflann.hpp"
#include "map_point.h"
namespace nanoflann
{
struct MapPointAdaptor
{
    explicit MapPointAdaptor(std::vector<MapPoint *>& _vmp) : map_pc_(_vmp) { }
    inline size_t kdtree_get_point_count() const;
    inline double kdtree_get_pt(const size_t idx, int dim) const;
    template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
    std::vector<MapPoint *> map_pc_;
};

/**
 * KD-Tree of MapPoint. One time construction, cannot add or delete points, so have to construct a new KD-Tree every
 * time it is needed. Used like this in LOAM, not sure about the efficiency.
 */
class KDTreeMapPoint
{
public:
    explicit KDTreeMapPoint(std::vector<MapPoint *> &_vmp, bool _sorted = true);

    int radiusSearch (const Vector3d &point, double radius, std::vector<int> &k_indices,
            std::vector<double> &k_sqr_dist) const;

    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<double, MapPointAdaptor>,
        MapPointAdaptor,
        3
        > KDTreeFlann_map_point;

    MapPointAdaptor adaptor_;

    SearchParams params_;

    KDTreeFlann_map_point kdtree_;
};

/**
 * Dynamic KD-Tree of MapPoint, that allows you to add and remove points from the KD-Tree
 * You can do three things:
 * 1. Construct a tree
 * 2. Add points, you have to first add the points to the container that you are using, then call addPoints() where
 *    you have to pass in the container, and the start and end index ([start, end]).
 * 3. Remove single point, you have to remove a point by its index from both the container and the kd-tree using
 *    function removePoint()
 */
class KDTreeDynamicMapPoint
{
public:
    explicit KDTreeDynamicMapPoint(std::vector<MapPoint *> &_vmp, bool _sorted = true);

    int radiusSearch (const Vector3d &point, double radius, std::vector<int> &k_indices,
                      std::vector<double> &k_sqr_dist) const;

    bool addPoints(std::vector<MapPoint *> &_vmp, size_t start, size_t end);

    //template <typename IndexT>
    bool removePoint(size_t idx);

    typedef KDTreeSingleIndexDynamicAdaptor<
            L2_Simple_Adaptor<double, MapPointAdaptor>,
            MapPointAdaptor,
            3
    > KDTreeFlann_map_point;

    MapPointAdaptor adaptor_;

    SearchParams params_;

    KDTreeFlann_map_point kdtree_;

    static const size_t CHUNK_SIZE = 100;
};
}

#endif //POSE_GRAPH_NANOFLANN_MAP_POINT_H
