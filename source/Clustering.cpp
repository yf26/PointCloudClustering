//
// Created by yu on 12.05.20.
//

#include "Clustering.hpp"


Clustering::Clustering(pcl_pcd_t::Ptr &cloud)
{
    cloud_ = cloud;
}


std::vector<int> Clustering::getGroundIndices()
{
    return ground_indices_;
}


std::vector<int> Clustering::getClusterLabel()
{
    return cluster_label_;
}


void Clustering::extractInitialSeeds(pcl_pcd_t::Ptr& input_cloud, pcl_pcd_t::Ptr& init_cloud,
                                     int LPRSize, double thresholdSeeds)
{
    float z_high = -1.73 + 0.5;
    pcl::PassThrough<pcl::PointXYZ> z_filter;
    z_filter.setInputCloud(input_cloud);
    z_filter.setFilterFieldName("z");
    z_filter.setFilterLimits(-50, z_high);
    z_filter.filter(*init_cloud);


    float LPR_z = 0;
    if (LPRSize >= init_cloud->size())
    {
//        std::cout << "LPRSize too large!" << std::endl;
        for (const auto& point : init_cloud->points)
            LPR_z += point.z;
        LPR_z /= init_cloud->size();
    }
    else
    {
        std::priority_queue<float> pq;
        for (const auto& point : init_cloud->points)
        {
            if (pq.size() < LPRSize)
                pq.push(point.z);
            else if (pq.top() > point.z)
            {
                pq.pop();
                pq.push( point.z);
            }
        }
        float found_z = pq.top();

        for (const auto& point : init_cloud->points)
        {
            if (point.z < found_z)
                LPR_z += point.z;
        }
        LPR_z /= init_cloud->size();
    }

    float upper_bound = LPR_z + (float)thresholdSeeds;

    z_filter.setInputCloud(init_cloud);
    z_filter.setFilterFieldName("z");
    z_filter.setFilterLimits(-10, upper_bound);
    z_filter.filter(*init_cloud);
}

void Clustering::estimatePlane(pcl_pcd_t::Ptr& input_cloud, Eigen::Vector4f& params)
{
    int cloud_size = input_cloud->size();
    Eigen::MatrixXf points = input_cloud->getMatrixXfMap().block(0, 0, 3, cloud_size).transpose();
    Eigen::Vector3f center = points.colwise().mean();
    Eigen::MatrixXf centered_points = points.rowwise() - center.transpose();


//    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
//        centered_points.transpose() * centered_points
//    );
//    Eigen::Vector3f normal = eigen_solver.eigenvectors().col(0).real();

    Eigen::Matrix3d XTX = (centered_points.transpose() * centered_points).cast<double>();
    Eigen::Vector3f normal = FastEigen3x3(XTX).cast<float>();

    float d = - normal.dot(center);
    params.block(0, 0, 3, 1) = normal;
    params(3, 0) = d;
}


void Clustering::groundRemoval(pcl_pcd_t::Ptr& input_cloud, std::vector<int>& input_idx,
                               std::vector<int>& ground_idx, /*std::vector<int>& foreground_idx,*/
                               int LPRSize ,int max_iter, double threshold_dist)
{
    int cloud_size = input_cloud->size();
    Eigen::MatrixXf points = input_cloud->getMatrixXfMap().transpose();

    pcl_pcd_t::Ptr seeds(new pcl_pcd_t);
    extractInitialSeeds(input_cloud, seeds, LPRSize, threshold_dist);

    Eigen::Vector4f params;
    Eigen::MatrixXf dists;
    for (int iter = 0; iter < max_iter; iter++)
    {
        estimatePlane(seeds, params);
        dists = (points * params).cwiseAbs();
        seeds->points.clear();
        assert(dists.rows() == cloud_size);
        for (int i = 0; i < cloud_size; i++)
        {
            if (dists(i) < threshold_dist)
            {
                seeds->points.emplace_back(input_cloud->points[i]);
                if (iter == max_iter-1)
                    ground_idx.emplace_back(input_idx[i]);
            }
//            else
//            {
//                if (iter == max_iter-1)
//                    foreground_idx.emplace_back(input_idx[i]);
//            }
        }
    }
//    std::cout << "Ground params = " << params.transpose() << std::endl;
}


void Clustering::groundSeparation(int max_iter, double threshold_dist,
                                  pcl_pcd_t::Ptr& ground, pcl_pcd_t::Ptr& foreground)
{
    int cloud_size = cloud_->size();

    std::vector<int> input_idx;
    input_idx.reserve(cloud_size);
    std::iota(input_idx.begin(), input_idx.end(), 0);

    std::vector<int> ground_idx/*, foreground_idx*/;
    ground_idx.reserve(cloud_size);
//    foreground_idx.reserve(cloud_size);
    groundRemoval(cloud_, input_idx, ground_idx, 10000, max_iter, threshold_dist);
    ground_indices_ = ground_idx;
//    foreground_indices_ = foreground_idx;

    pcl::PointIndices::Ptr ground_filter (new pcl::PointIndices);
    ground_filter->indices = ground_idx;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_);
    extract.setIndices(ground_filter);
    extract.setNegative(false);
    extract.filter(*ground);
    extract.setNegative(true);
    extract.filter(*foreground);

    std::cout << "Ground size " << ground->size() << std::endl;

    ///
    // TODO filter method copy or reference
    ///
}


void Clustering::groundSeparationV3(int max_iter, double threshold_dist, float main_dist,
                                    pcl_pcd_t::Ptr& ground, pcl_pcd_t::Ptr& foreground)
{
    pcl::PassThrough<pcl::PointXYZ> x_partition_filter;
    x_partition_filter.setInputCloud(cloud_);
    x_partition_filter.setFilterFieldName("x");

    float partition[4] = {-200, -main_dist, main_dist, 200};
    std::vector<int> ground_idx;

    for (int i = 0; i < 3; i++)
    {
        pcl_pcd_t::Ptr cloud_seg(new pcl_pcd_t);
        std::vector<int> cloud_seg_idx;
        x_partition_filter.setFilterLimits(partition[i], partition[i+1]);
        x_partition_filter.filter(*cloud_seg);
        x_partition_filter.filter(cloud_seg_idx);

//        ///
//        std::cout << "cloud_seg_idx size = " << cloud_seg_idx.size() << std::endl;
//        ///

        std::vector<int> ground_seg_idx;
        groundRemoval(cloud_seg, cloud_seg_idx, ground_seg_idx, 10000, max_iter, threshold_dist);

        // concatenate all ground_seg_idx in ground_idx
        ground_idx.reserve(ground_idx.size() + ground_seg_idx.size());
        ground_idx.insert(ground_idx.end(), ground_seg_idx.begin(), ground_seg_idx.end());
    }

    ground_indices_ = ground_idx;

    pcl::PointIndices::Ptr ground_filter (new pcl::PointIndices);
    ground_filter->indices = ground_idx;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_);
    extract.setIndices(ground_filter);
    extract.setNegative(false);
    extract.filter(*ground);
    extract.setNegative(true);
    extract.filter(*foreground);

    foreground_cloud_ = foreground;

}


void Clustering::DBSCAN(float eps, int minPts)
{
#define NOISE -1
#define UNDEFINED 0

    int cloud_size = foreground_cloud_->size();
    std::vector<std::vector<float>> points(cloud_size, std::vector<float>(3));

    // pcl_pcd_t to vec<vec> point cloud type
    for (int i = 0; i < cloud_size; i++)
        points[i] = std::vector<float> {
            foreground_cloud_->points[i].x, foreground_cloud_->points[i].y, foreground_cloud_->points[i].z
        };

    int C = UNDEFINED;
    cluster_label_.reserve(cloud_size);
    cluster_label_ = std::vector<int>(cloud_size, C);

    // build kdtree for foreground points
    float radius = eps;
    int leaf_size = 10;
    Node* root = KDTreeConstruction(points, leaf_size);

    for (int i = 0; i < cloud_size; i++)
    {
        if (cluster_label_[i] != UNDEFINED) continue;
        RadiusNNResultSet result_set(radius);
        KDTreeRadiusNNSearch(root, points, result_set, points[i]);

        int nn_numbers = result_set.size();
        assert(nn_numbers == result_set.distIndexList.size());

        if (nn_numbers < minPts)
        {
            cluster_label_[i] = NOISE;
            continue;
        }

        C++;
        cluster_label_[i] = C;

        std::vector<int> seed_set; seed_set.reserve(nn_numbers - 1);
        for (const auto& item : result_set.distIndexList)
        {
            if (item.index == i) continue;
            seed_set.emplace_back(item.index);
        }

        ///
        expandCluster(seed_set, C, minPts, radius, root, points);
        ///

    }

}


void Clustering::expandCluster(const std::vector<int>& seed_set, const int& C,
                               const int& minPts, const float& radius,
                               Node*& root, std::vector<std::vector<float>>& points)
{
    for (const auto& idx: seed_set)
    {
        if (cluster_label_[idx] == NOISE) cluster_label_[idx] = C;
        else if (cluster_label_[idx] != UNDEFINED) continue;
        else if (cluster_label_[idx] == UNDEFINED)
        {
            cluster_label_[idx] = C;

            RadiusNNResultSet result_set(radius);
            KDTreeRadiusNNSearch(root, points, result_set, points[idx]);

            int nn_numbers = result_set.size();
            if (nn_numbers >= minPts)
            {
                // TODO recursive
                std::vector<int> new_seed_set; new_seed_set.reserve(nn_numbers - 1);
                for (const auto& item : result_set.distIndexList)
                {
                    if (item.index == idx) continue;
                    new_seed_set.emplace_back(item.index);
                }
                // expandCluster on this new seed set
                expandCluster(new_seed_set, C, minPts, radius, root, points);
            }
        }
    }
}














