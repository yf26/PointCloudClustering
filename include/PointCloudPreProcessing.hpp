//
// Created by yu on 12.05.20.
//

#ifndef HW4_POINTCLOUDPREPROCESSING_HPP
#define HW4_POINTCLOUDPREPROCESSING_HPP
#include <chrono>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZ> pcl_pcd_t;

void readBinary(const std::string& fileName, pcl_pcd_t::Ptr& cloud, float y_left, float y_right, int nn, double std_ratio);


#endif //HW4_POINTCLOUDPREPROCESSING_HPP
