#ifndef _ASN1SCC_EIGEN_CONVERSIONS_H_
#define _ASN1SCC_EIGEN_CONVERSIONS_H_

// -----------
// Eigen types 
// -----------
#include <Eigen/Geometry>

// -----------
// ASN1 types 
// -----------
#include <Pose.h>


void toASN1SCC(const Eigen::Vector3f& eigen_vector, Position& pos);

void toASN1SCC(const Eigen::Matrix3f& eigen_rot_matrix, Orientation& orient);

void toASN1SCC(const Eigen::Affine3f& eigen_transform, Pose& pose);

#endif // _ASN1SCC_EIGEN_CONVERSIONS_H_