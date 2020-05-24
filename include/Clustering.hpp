//
// Created by yu on 12.05.20.
//

#ifndef HW4_CLUSTERING_HPP
#define HW4_CLUSTERING_HPP
#include <algorithm>
#include <queue>
#include <numeric>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include "fastEigen.hpp"
#include "resultSet.hpp"
#include "kdtree.hpp"


typedef pcl::PointCloud<pcl::PointXYZ> pcl_pcd_t;

class Clustering
{
private:
    pcl_pcd_t::Ptr cloud_;
    std::vector<int> ground_indices_;
    pcl_pcd_t::Ptr foreground_cloud_;
    std::vector<int> cluster_label_;

    void extractInitialSeeds(pcl_pcd_t::Ptr& input_cloud, pcl_pcd_t::Ptr& init_cloud,
                                         int LPRSize, double thresholdSeeds);
    void estimatePlane(pcl_pcd_t::Ptr& input_cloud, Eigen::Vector4f& params);
    void groundRemoval(pcl_pcd_t::Ptr& input_cloud, std::vector<int>& input_idx,
                       std::vector<int>& ground_idx, /*std::vector<int>& foreground_idx,*/
                       int LPRSize ,int max_iter, double threshold_dist);
    void expandCluster(const std::vector<int>& seed_set, const int& C,
                       const int& minPts, const float& radius,
                       Node*& root, std::vector<std::vector<float>>& points);

public:
    Clustering(pcl_pcd_t::Ptr& cloud);
    std::vector<int> getGroundIndices();
    std::vector<int>  getClusterLabel();

    void groundSeparation(int max_iter, double threshold_dist,
                         pcl_pcd_t::Ptr& ground, pcl_pcd_t::Ptr& foreground);
    void groundSeparationV3(int max_iter, double threshold_dist, float main_dist,
                            pcl_pcd_t::Ptr& ground, pcl_pcd_t::Ptr& foreground);
    void DBSCAN(float eps, int minPts);

};


#endif //HW4_CLUSTERING_HPP
