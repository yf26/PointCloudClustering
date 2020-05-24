//
// Created by yu on 13.05.20.
//

#ifndef HW4_FASTEIGEN_HPP
#define HW4_FASTEIGEN_HPP
#include <Eigen/Eigenvalues>
Eigen::Vector3d ComputeEigenvector0(const Eigen::Matrix3d &A, double eval0);
Eigen::Vector3d ComputeEigenvector1(const Eigen::Matrix3d &A,
                                    const Eigen::Vector3d &evec0,
                                    double eval1);
Eigen::Vector3d FastEigen3x3(Eigen::Matrix3d &A);
#endif //HW4_FASTEIGEN_HPP
