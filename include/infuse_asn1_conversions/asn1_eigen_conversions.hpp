#ifndef _ASN1SCC_EIGEN_CONVERSIONS_H_
#define _ASN1SCC_EIGEN_CONVERSIONS_H_

// -----------
// Eigen types 
// -----------
#include <Eigen/Geometry>

// -----------
// ASN1 types 
// -----------
#include <infuse_asn1_types/Pose.h>
#include <infuse_asn1_types/TransformWithCovariance.h>


void toASN1SCC(const Eigen::Vector3f& eigen_vector, asn1SccPosition& pos);

void fromASN1SCC(const asn1SccPosition& pos, Eigen::Vector3f& eigen_vector);

void toASN1SCC(const Eigen::Quaternionf& quat, asn1SccOrientation& orient);

void fromASN1SCC(const asn1SccOrientation& orient, Eigen::Quaternionf& quat);

void toASN1SCC(const Eigen::Matrix3f& eigen_rot_matrix, asn1SccOrientation& orient);

void fromASN1SCC(const asn1SccOrientation& orient, Eigen::Matrix3f& eigen_rot_matrix);

void toASN1SCC(const Eigen::Affine3f& eigen_transform, asn1SccPose& pose);

template<class EIGEN_TRANSFORM>
void toASN1SCC(const EIGEN_TRANSFORM& eigen_transform, asn1SccTransformWithCovariance& transform)
{
	// NOTE: transform.cov not set in this function!

	typename EIGEN_TRANSFORM::VectorType t(eigen_transform.translation());
	transform.data.translation.arr[0] = t[0];
	transform.data.translation.arr[1] = t[1];
	transform.data.translation.arr[2] = t[2];

	Eigen::Quaternion<typename EIGEN_TRANSFORM::Scalar> quat(eigen_transform.rotation());
	transform.data.orientation.arr[0] = quat.x();
	transform.data.orientation.arr[1] = quat.y();
	transform.data.orientation.arr[2] = quat.z();
	transform.data.orientation.arr[3] = quat.w();
}


// TODO: Implement this function
// void fromASN1SCC(const Pose& pose, Eigen::Affine3f& eigen_transform);


#endif // _ASN1SCC_EIGEN_CONVERSIONS_H_