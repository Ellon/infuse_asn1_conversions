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
#include <TransformWithCovariance.h>


void toASN1SCC(const Eigen::Vector3f& eigen_vector, Position& pos);

void fromASN1SCC(const Position& pos, Eigen::Vector3f& eigen_vector);

void toASN1SCC(const Eigen::Quaternionf& quat, Orientation& orient);

void fromASN1SCC(const Orientation& orient, Eigen::Quaternionf& quat);

void toASN1SCC(const Eigen::Matrix3f& eigen_rot_matrix, Orientation& orient);

void fromASN1SCC(const Orientation& orient, Eigen::Matrix3f& eigen_rot_matrix);

void toASN1SCC(const Eigen::Affine3f& eigen_transform, Pose& pose);

template<class EIGEN_TRANSFORM>
void toASN1SCC(const EIGEN_TRANSFORM& eigen_transform, TransformWithCovariance& transform)
{
	// NOTE: transform.cov not set in this function!

	typename EIGEN_TRANSFORM::VectorType t(eigen_transform.translation());
	transform.translation.arr[0] = t[0];
	transform.translation.arr[1] = t[1];
	transform.translation.arr[2] = t[2];
	transform.translation.nCount = 3;

	Eigen::Quaternion<typename EIGEN_TRANSFORM::Scalar> quat(eigen_transform.rotation());
	transform.orientation.arr[0] = quat.x();
	transform.orientation.arr[1] = quat.y();
	transform.orientation.arr[2] = quat.z();
	transform.orientation.arr[3] = quat.w();
	transform.orientation.nCount = 4;
}


// TODO: Implement this function
// void fromASN1SCC(const Pose& pose, Eigen::Affine3f& eigen_transform);


#endif // _ASN1SCC_EIGEN_CONVERSIONS_H_