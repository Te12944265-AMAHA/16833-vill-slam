//
// Created by dcheng on 10/28/19.
//

#include "nanoflann_map_point.h"

namespace nanoflann
{

KDTreeMapPoint::KDTreeMapPoint(std::vector<MapPoint *> &_vmp, bool _sorted)
: adaptor_(_vmp)
, kdtree_(3, adaptor_)
{
    params_.sorted = _sorted;
    kdtree_.buildIndex();
}

int KDTreeMapPoint::radiusSearch(const Vector3d &point, double radius, std::vector<int> &k_indices,
                                 std::vector<double> &k_sqr_dist) const
{
    static std::vector<std::pair<int, double>> indices_dist;
    indices_dist.reserve(256);
    RadiusResultSet<double, int> result_set(radius, indices_dist);
    double v_point[3] = {point[0], point[1], point[2]};
    kdtree_.findNeighbors(result_set, v_point, params_);

    const size_t n_found = indices_dist.size();

    if (params_.sorted)
        std::sort(indices_dist.begin(), indices_dist.end(), IndexDist_Sorter());

    k_indices.resize(n_found);
    k_sqr_dist.resize(n_found);
    //std::cout << "Result within class\n";
    for (int i = 0; i < n_found; i++)
    {
        k_indices[i]  = indices_dist[i].first;
        k_sqr_dist[i] = indices_dist[i].second;
        //std::cout << indices_dist[i].first << ", " << indices_dist[i].second << std::endl;
    }
    //std::cout << "finished result within class \n";

    return n_found;
}

size_t MapPointAdaptor::kdtree_get_point_count() const
{
    return map_pc_.size();
}

double MapPointAdaptor::kdtree_get_pt(const size_t idx, int dim) const
{
    return map_pc_[idx]->world_pos_[dim];
}

KDTreeDynamicMapPoint::KDTreeDynamicMapPoint(std::vector<MapPoint *> &_vmp, bool _sorted)
: adaptor_(_vmp)
, kdtree_(3, adaptor_)
{
    params_.sorted = _sorted;
    // size_t n_points = _vmp.size();
}

int KDTreeDynamicMapPoint::radiusSearch(const Vector3d &point, double radius, std::vector<int> &k_indices,
                                        std::vector<double> &k_sqr_dist) const
{
    //cout << "search size: " << adaptor_.map_pc_.size() << endl;
    static std::vector<std::pair<int, double>> indices_dist;
    indices_dist.reserve(256);
    RadiusResultSet<double, int> result_set(radius, indices_dist);
    double v_point[3] = {point[0], point[1], point[2]};
    kdtree_.findNeighbors(result_set, v_point, params_);

    const size_t n_found = indices_dist.size();

    if (params_.sorted)
        std::sort(indices_dist.begin(), indices_dist.end(), IndexDist_Sorter());

    k_indices.resize(n_found);
    k_sqr_dist.resize(n_found);
    //std::cout << "Result within class\n";
    for (int i = 0; i < n_found; i++)
    {
        k_indices[i]  = indices_dist[i].first;
        k_sqr_dist[i] = indices_dist[i].second;
        //std::cout << indices_dist[i].first << ", " << indices_dist[i].second << std::endl;
    }
    //std::cout << "finished result within class \n";

    return n_found;
}

bool KDTreeDynamicMapPoint::addPoints(std::vector<MapPoint *> &_vmp, size_t start, size_t end)
{
    adaptor_.map_pc_ = _vmp;
    cout << "new size: " << adaptor_.map_pc_.size() << endl;
    // add the points in chunks
    for (size_t i = 0; i < (end - start + 1); i += CHUNK_SIZE)
    {
        size_t iter_end = std::min(size_t(i + CHUNK_SIZE), end - start) + start - 1;
        kdtree_.addPoints(start + i, iter_end);
    }

    return true;
}

//template<typename size_t>
bool KDTreeDynamicMapPoint::removePoint(size_t idx)
{
    kdtree_.removePoint(idx);
    return true;
}

}