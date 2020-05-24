//
// Created by yu on 13.05.20.
//
#include "kdtree.hpp"


int updateAxis(int axis, int dim)
{
//    std::cout << "Update axis = " << axis << std::endl;
//    std::cout << "dim = " << dim << std::endl;
    if (axis == dim - 1)
        return 0;
    else if (axis < dim)
        return axis + 1;
}


/*
 *  This Quickselect routine is based on the algorithm described in
 *  "Numerical recipes in C", Second Edition,
 *  Cambridge University Press, 1992, Section 8.5, ISBN 0-521-43108-5
 *  This code by Nicolas Devillard - 1998. Public domain.
 *  ATTENTION: the input data set must be copied prior to applying the median search.
 */

template <typename elem_type>
#define ELEM_SWAP(a,b) { register elem_type t=a; a=b;b=t; }
elem_type quickSelect(std::vector<elem_type> arr, int n)
{
    int low, high ;
    int median;
    int middle, ll, hh;

    low = 0 ; high = n-1 ; median = (low + high) / 2;
    for (;;)
    {
        if (high <= low) /* One element only */
            return arr[median] ;

        if (high == low + 1) {  /* Two elements only */
            if (arr[low] > arr[high])
            ELEM_SWAP(arr[low], arr[high]) ;
            return arr[median] ;
        }

        /* Find median of low, middle and high items; swap into position low */
        middle = (low + high) / 2;
        if (arr[middle] > arr[high])    ELEM_SWAP(arr[middle], arr[high]) ;
        if (arr[low] > arr[high])       ELEM_SWAP(arr[low], arr[high]) ;
        if (arr[middle] > arr[low])     ELEM_SWAP(arr[middle], arr[low]) ;

        /* Swap low item (now in position middle) into position (low+1) */
        ELEM_SWAP(arr[middle], arr[low+1]) ;

        /* Nibble from each end towards middle, swapping items when stuck */
        ll = low + 1;
        hh = high;
        for (;;) {
            do ll++; while (arr[low] > arr[ll]) ;
            do hh--; while (arr[hh]  > arr[low]) ;

            if (hh < ll)
                break;

            ELEM_SWAP(arr[ll], arr[hh]) ;
        }

        /* Swap middle item (in position low) back into correct position */
        ELEM_SWAP(arr[low], arr[hh]) ;

        /* Re-set active partition */
        if (hh <= median)
            low = ll;
        if (hh >= median)
            high = hh - 1;
    }

}
#undef ELEM_SWAP


std::vector<int> argsort(std::vector<ElemType>& arr)
{
    int n = arr.size();
    std::vector<int> idx(n);
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&arr](int i, int j) {
        return arr[i] < arr[j];
    });
    return idx;
}



void sortKeysbyValues(const std::vector<int>& point_indices,
                      const std::vector<ElemType>& values,
                      std::vector<int>& key_sorted, std::vector<ElemType>& value_sorted)
{

    value_sorted = values;
    std::vector<int> idx_sorted = argsort(value_sorted);
    value_sorted.clear();

    for (auto& idx : idx_sorted)
    {
        key_sorted.emplace_back(point_indices[idx]);
        value_sorted.emplace_back(values[idx]);
    }

}


Node* KDTreeBuild(Node*& root,
                  std::vector<std::vector<ElemType>>& db,
                  std::vector<int>& point_indices,
                  int axis, int leaf_size)
{
    ///
    std::cout << "Build axis = " << axis << std::endl;
    ///

    if (root == nullptr)
    {
        // TODO
        root = new Node(axis, NONE, nullptr, nullptr, point_indices);
        Node::address_set.emplace_back(root);
    }

    if (point_indices.size() > leaf_size)
    {

        std::vector<ElemType> dbCrop;
        dbCrop.reserve(point_indices.size()); // TODO: test speed
        for (auto& item : point_indices)
            dbCrop.emplace_back(db[item][axis]);


        std::vector<int> point_indices_sorted;
        point_indices_sorted.reserve(point_indices.size());
        std::vector<ElemType> value_sorted;
        value_sorted.reserve(point_indices.size());

        sortKeysbyValues(point_indices, dbCrop, point_indices_sorted, value_sorted);

        int middle_left_pos = (int)ceil(point_indices_sorted.size() / 2.) - 1;
        int middle_right_pos = middle_left_pos + 1;
        int middle_left_idx = point_indices_sorted[middle_left_pos];
        int middle_right_idx = point_indices_sorted[middle_right_pos];
        root->value = (db[middle_left_idx][axis] + db[middle_right_idx][axis]) * 0.5;

        std::vector<int> point_idx_left = std::vector<int>(
            point_indices_sorted.begin(), point_indices_sorted.begin() + middle_right_pos
        );
        std::vector<int> point_idx_right = std::vector<int>(
            point_indices_sorted.begin() + middle_right_pos, point_indices_sorted.end()
        );


        // build left subtree for node above
        root->left = KDTreeBuild(
            root->left, db, point_idx_left,
            updateAxis(axis, (db[0]).size()), leaf_size
        );

        // build right subtree for node above
        root->right = KDTreeBuild(
            root->right, db, point_idx_right,
            updateAxis(axis, (db[0]).size()), leaf_size
        );

    }

    return root;
}


Node* KDTreeBuildFastMedian(Node*& root,
                            std::vector<std::vector<ElemType>>& db,
                            std::vector<int>& point_indices,
                            int axis, int leaf_size)
{
//    ///
//    std::cout << "Build axis = " << axis << std::endl;
//    ///

    if (root == nullptr)
    {
        // TODO
        root = new Node(axis, NONE, nullptr, nullptr, point_indices);
        Node::address_set.emplace_back(root);
    }

    if (point_indices.size() > leaf_size)
    {
        std::vector<ElemType> dbCrop;
        dbCrop.reserve(point_indices.size()); // TODO: test speed
        for (auto& item : point_indices)
            dbCrop.emplace_back(db[item][axis]);


        ElemType median = quickSelect(dbCrop, dbCrop.size());
        root->value = median;

        std::vector<int> point_idx_left;
        std::vector<int> point_idx_right;
        for (auto& idx : point_indices)
        {
            if (db[idx][axis] <= median)
                point_idx_left.emplace_back(idx);
            else
                point_idx_right.emplace_back(idx);
        }


        // build left subtree for node above
        root->left = KDTreeBuildFastMedian(
            root->left, db, point_idx_left,
            updateAxis(axis, (db[0]).size()), leaf_size
        );

        // build right subtree for node above
        root->right = KDTreeBuildFastMedian(
            root->right, db, point_idx_right,
            updateAxis(axis, (db[0]).size()), leaf_size
        );

    }

    return root;
}


Node* KDTreeBuildFastMean(Node*& root,
                          std::vector<std::vector<ElemType>>& db,
                          std::vector<int>& point_indices,
                          int axis, int leaf_size)
{
    if (root == nullptr)
    {
        // TODO
        root = new Node(axis, NONE, nullptr, nullptr, point_indices);
        Node::address_set.emplace_back(root);
    }

    if (point_indices.size() > leaf_size)
    {
        std::vector<ElemType> dbCrop;
        dbCrop.reserve(point_indices.size()); // TODO: test speed
        for (auto& item : point_indices)
            dbCrop.emplace_back(db[item][axis]);

//        ///
//        std::cout << "Crop size = " << dbCrop.size() << std::endl;
//        std::cout << "Indices size = " << point_indices.size() << std::endl;
//        ///

        ElemType mean = 0;
        for (auto& item : dbCrop)
            mean += item;
        mean = mean / dbCrop.size();

//        ///
//        std::cout << mean << std::endl;
//        ///

        root->value = mean;


        std::vector<int> point_idx_left;
        std::vector<int> point_idx_right;
        for (auto& idx : point_indices)
        {
            if (db[idx][axis] <= mean)
                point_idx_left.emplace_back(idx);
            else
                point_idx_right.emplace_back(idx);
        }


        // build left subtree for node above
        root->left = KDTreeBuildFastMean(
            root->left, db, point_idx_left,
            updateAxis(axis, (db[0]).size()), leaf_size
        );

        // build right subtree for node above
        root->right = KDTreeBuildFastMean(
            root->right, db, point_idx_right,
            updateAxis(axis, (db[0]).size()), leaf_size
        );

    }

    return root;
}


void KDTreeKNNSearch(Node*& root,
                     std::vector<std::vector<ElemType>>& db,
                     KNNResultSet& result_set,
                     std::vector<ElemType>& query)
{
    if (root == nullptr)
        return;

    if (root->isLeaf()) // TODO
    {
        for (auto& idx : root->point_indices)
        {
            ElemType diff = 0;
            for (int i = 0; i < db[0].size(); i ++)
            {
                diff += pow(db[idx][i] - query[i], 2);
            }
            diff = sqrt(diff);
            result_set.addPoint(diff, idx);
        }
        return;
    }

    if (query[root->axis] <= root->value)
    {
        KDTreeKNNSearch(root->left, db, result_set, query);
        if ( fabs(query[root->axis] - root->value) < result_set.getWorstDist() )
            KDTreeKNNSearch(root->right, db, result_set, query);
    }
    else
    {
        KDTreeKNNSearch(root->right, db, result_set, query);
        if ( fabs(query[root->axis] - root->value) < result_set.getWorstDist() )
            KDTreeKNNSearch(root->left, db, result_set, query);
    }
}


void KDTreeRadiusNNSearch(Node*& root,
                          std::vector<std::vector<ElemType>>& db,
                          RadiusNNResultSet& result_set,
                          std::vector<ElemType>& query)
{
    if (root == nullptr)
        return;

    if (root->isLeaf()) // TODO
    {
        for (auto& idx : root->point_indices)
        {
            ElemType diff = 0;
            for (int i = 0; i < db[0].size(); i ++)
            {
                diff += pow(db[idx][i] - query[i], 2);
            }
            diff = sqrt(diff);
            result_set.addPoint(diff, idx);
        }
        return;
    }

    if (query[root->axis] <= root->value)
    {
        KDTreeRadiusNNSearch(root->left, db, result_set, query);
        if ( fabs(query[root->axis] - root->value) < result_set.getWorstDist() )
            KDTreeRadiusNNSearch(root->right, db, result_set, query);
    }
    else
    {
        KDTreeRadiusNNSearch(root->right, db, result_set, query);
        if ( fabs(query[root->axis] - root->value) < result_set.getWorstDist() )
            KDTreeRadiusNNSearch(root->left, db, result_set, query);
    }
}


int TreeDepth(Node*& root)
{
    if (root == nullptr)
        return 0;
    else
    {
        int d_left = TreeDepth(root->left);
        int d_right = TreeDepth(root->right);
        return std::max(d_left + 1, d_right + 1);
    }
}


std::vector<Node*> Node::address_set;
Node* KDTreeConstruction(std::vector<std::vector<ElemType>>& db, int leaf_size)
{
    int N = db.size();
    std::vector<int> point_indices(N);
    std::iota(point_indices.begin(), point_indices.end(), 0);

    Node* root = nullptr;
    root = KDTreeBuildFastMedian(root, db, point_indices, 0, leaf_size);
//    std::cout << Node::address_set.size() << " Nodes address are saved!" << std::endl;
    return root;
}

void KDTreeDestruction()
{
    for (auto& item: Node::address_set)
        delete item;
}