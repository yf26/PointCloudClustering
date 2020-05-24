cmake_minimum_required(VERSION 3.15)
project(hw4)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(/usr/local/include/eigen3/)
include_directories(./include ./source)

add_executable (hw4 main.cpp source/PointCloudPreProcessing.cpp include/PointCloudPreProcessing.hpp source/Clustering.cpp include/Clustering.hpp source/fastEigen.cpp include/fastEigen.hpp include/kdtree.hpp include/resultSet.hpp source/kdtree.cpp source/resultSet.cpp)
target_link_libraries(hw4 ${PCL_LIBRARIES})