#ifndef HW2_KDTREE_HPP
#define HW2_KDTREE_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include "resultSet.hpp"

#define NONE 1e10

#define ElemType float

class Node
{
public:
    int axis;
    Node* left;
    Node* right;
    std::vector<int> point_indices;
    ElemType value;

    Node(int& ax, ElemType v, Node* l, Node* r, std::vector<int>& point_idx)
    {
        axis = ax;
        value = v;
        left = l;
        right = r;
        point_indices = point_idx;
    }

    bool isLeaf()
    {
        return value == NONE;
    }

    static std::vector<Node*> address_set;

};


int updateAxis(int axis, int dim);


std::vector<int> argsort(std::vector<ElemType>& arr);



void sortKeysbyValues(const std::vector<int>& point_indices,
                      const std::vector<ElemType>& values,
                      std::vector<int>& key_sorted, std::vector<ElemType>& value_sorted);


Node* KDTreeBuild(Node*& root,
                  std::vector<std::vector<ElemType>>& db,
                  std::vector<int>& point_indices,
                  int axis, int leaf_size);


Node* KDTreeBuildFastMedian(Node*& root,
                            std::vector<std::vector<ElemType>>& db,
                            std::vector<int>& point_indices,
                            int axis, int leaf_size);


Node* KDTreeBuildFastMean(Node*& root,
                      std::vector<std::vector<ElemType>>& db,
                      std::vector<int>& point_indices,
                      int axis, int leaf_size);


void KDTreeKNNSearch(Node*& root,
                     std::vector<std::vector<ElemType>>& db,
                     KNNResultSet& result_set,
                     std::vector<ElemType>& query);


void KDTreeRadiusNNSearch(Node*& root,
                          std::vector<std::vector<ElemType>>& db,
                          RadiusNNResultSet& result_set,
                          std::vector<ElemType>& query);


int TreeDepth(Node*& root);


Node* KDTreeConstruction(std::vector<std::vector<ElemType>>& db, int leaf_size);

void KDTreeDestruction();

#endif //HW2_KDTREE_HPP