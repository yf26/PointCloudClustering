//
// Created by yu on 12.05.20.
//
#include "PointCloudPreProcessing.hpp"

void readBinary(const std::string& fileName, pcl_pcd_t::Ptr& cloud, float y_left, float y_right, int nn, double std_ratio)
{
    std::fstream input(fileName.c_str(), std::ios::in | std::ios::binary);
    input.seekg(0, std::ios::beg);
    pcl_pcd_t::Ptr raw_cloud(new pcl_pcd_t);

    if (not input.good())
    {
        std::cerr << "Read file " << fileName << "failed!";
        exit(EXIT_FAILURE);
    }

    for (int i = 0; input.good() and !input.eof(); i++)
    {
        float temp[4];
        input.read((char* )&temp, 4*sizeof(float));
//        if (temp[1] < y_right or temp[1] > y_left)
//            continue;
        raw_cloud->points.emplace_back(temp[0], temp[1], temp[2]);
    }
    input.close();
    std::cout << raw_cloud->points.size() << " points read!" << std::endl;

//    pcl::ConditionAnd<pcl::PointXYZ>::Ptr y_cond (new pcl::ConditionAnd<pcl::PointXYZ>());
//    y_cond->addComparison(
//        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
//            new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, y_left)
//        )
//    );
//    y_cond->addComparison(
//        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
//            new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, y_right)
//        )
//    );
//    pcl::ConditionalRemoval<pcl::PointXYZ> cond_rem;
//    cond_rem.setCondition(y_cond);
//    cond_rem.setInputCloud(raw_cloud);
//    cond_rem.setKeepOrganized(false);
//    cond_rem.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> y_filter;
    y_filter.setInputCloud(raw_cloud);
    y_filter.setFilterFieldName("y");
    y_filter.setFilterLimits(y_right, y_left);
    y_filter.filter(*cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(nn);
    sor.setStddevMulThresh(std_ratio);
    sor.filter(*cloud);

    std::cout << "Filtered pcd size " << cloud->size() << std::endl;
}
