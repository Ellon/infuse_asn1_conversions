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

void fromASN1SCC(const Position& pos, Eigen::Vector3f& eigen_vector);

void toASN1SCC(const Eigen::Quaternionf& quat, Orientation& orient);

void fromASN1SCC(const Orientation& orient, Eigen::Quaternionf& quat);

void toASN1SCC(const Eigen::Matrix3f& eigen_rot_matrix, Orientation& orient);

void fromASN1SCC(const Orientation& orient, Eigen::Matrix3f& eigen_rot_matrix);

void toASN1SCC(const Eigen::Affine3f& eigen_transform, Pose& pose);

// TODO: Implement this function
// void fromASN1SCC(const Pose& pose, Eigen::Affine3f& eigen_transform);


#endif // _ASN1SCC_EIGEN_CONVERSIONS_H_