//
// Created by yu on 12.05.20.
//
#include <thread>
#include <chrono>
#include <iostream>
#include "include/PointCloudPreProcessing.hpp"
#include "include/Clustering.hpp"
#include "pcl/visualization/pcl_visualizer.h"

using namespace std::chrono_literals;

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::duration<double, std::milli> Duration;

int main()
{
    auto read_start = Clock::now();
    std::string file_name = "../../test/000099.bin";
    pcl_pcd_t::Ptr cloud ( new pcl_pcd_t );
    readBinary(file_name, cloud, 30, -15, 20, 2.7);
    auto read_end = Clock::now();
    Duration read_time = read_end - read_start;
    std::cout << "Reading and preprocessing takes " << read_time.count() << " ms" << std::endl;

    auto groundre_start = Clock::now();
    pcl_pcd_t::Ptr ground(new pcl_pcd_t), foreground(new pcl_pcd_t);
    Clustering cluster(cloud);
//    cluster.groundSeparation(10, 0.15, ground, foreground);
    cluster.groundSeparationV3(5, 0.18, 20, ground, foreground);
    auto groundre_end = Clock::now();
    Duration groundre_time = groundre_end - groundre_start;
    std::cout << "Ground pcd size " << ground->size() << std::endl;
    std::cout << "Foreground pcd size " << foreground->size() << std::endl;
    std::cout << "Ground Detection takes " << groundre_time.count() << " ms" << std::endl;



    auto cluster_start = Clock::now();
    cluster.DBSCAN(0.5, 8);
    auto clusters = cluster.getClusterLabel();
    auto cluster_end = Clock::now();
    Duration cluster_time = cluster_end - cluster_start;
    std::cout << "Clustering takes " << cluster_time.count() << " ms" << std::endl;


    // create new point cloud with rgb for visualization
    std::vector<std::vector<uint8_t>> colors(9, std::vector<uint8_t>(3));
    colors[0] = std::vector<uint8_t>{55, 126, 184};
    colors[1] = std::vector<uint8_t>{255, 127, 0};
    colors[2] = std::vector<uint8_t>{77, 175, 74};
    colors[3] = std::vector<uint8_t>{247, 129, 191};
    colors[4] = std::vector<uint8_t>{166, 86, 40};
    colors[5] = std::vector<uint8_t>{152, 78, 163};
    colors[6] = std::vector<uint8_t>{153, 153, 153};
    colors[7] = std::vector<uint8_t>{228, 26, 28};
    colors[8] = std::vector<uint8_t>{222, 222, 0};


    int fg_size = foreground->size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fg_clustered (new pcl::PointCloud<pcl::PointXYZRGB>);
    fg_clustered->points.reserve(fg_size);

    for (int i = 0; i < fg_size; i++)
    {
        pcl::PointXYZRGB point;
        point.x = foreground->points[i].x;
        point.y = foreground->points[i].y;
        point.z = foreground->points[i].z;
        if (clusters[i] == -1)
        {
            point.r = uint8_t(255);
            point.g = uint8_t(255);
            point.b = uint8_t(255);
        }
        else
        {
            int idx = clusters[i] % 9;
            point.r = colors[idx][0];
            point.g = colors[idx][1];
            point.b = colors[idx][2];
        }
        fg_clustered->points.emplace_back(point);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addPointCloud(fg_clustered, "foreground_clustered");
    viewer->addPointCloud(ground, "ground");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0., 0., 1., "ground");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
    return 0;
}
