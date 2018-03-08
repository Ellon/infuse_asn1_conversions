#include "conversions/asn1_eigen_conversions.hpp"

void toASN1SCC(const Eigen::Vector3f& eigen_vector, Position& pos)
{
	pos.arr[0] = eigen_vector[0];
	pos.arr[1] = eigen_vector[1];
	pos.arr[2] = eigen_vector[2];
	pos.nCount = 3;
}

void toASN1SCC(const Eigen::Matrix3f& eigen_rot_matrix, Orientation& orient)
{
	Eigen::Quaternionf quat(eigen_rot_matrix);
	// In a Eigen Quaternion the coefficients are stored internally in the
	// following order: [x, y, z, w], so we use the same order here.
	// See: https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#ad30f4da9a2c0c8dd95520ee8a6d14ef6
	orient.arr[0] = quat.x();
	orient.arr[1] = quat.y();
	orient.arr[2] = quat.z();
	orient.arr[3] = quat.w();
	orient.nCount = 4;
}

void toASN1SCC(const Eigen::Affine3f& eigen_transform, Pose& pose)
{
	toASN1SCC(eigen_transform.translation(), pose.pos);
	toASN1SCC(eigen_transform.rotation(), pose.orient);
}
