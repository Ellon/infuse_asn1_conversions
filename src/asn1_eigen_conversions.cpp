#include "infuse_asn1_conversions/asn1_eigen_conversions.hpp"

void toASN1SCC(const Eigen::Vector3f& eigen_vector, asn1SccPosition& pos)
{
	pos.arr[0] = eigen_vector[0];
	pos.arr[1] = eigen_vector[1];
	pos.arr[2] = eigen_vector[2];
}

void fromASN1SCC(const asn1SccPosition& pos, Eigen::Vector3f& eigen_vector)
{
	eigen_vector[0] = pos.arr[0];
	eigen_vector[1] = pos.arr[1];
	eigen_vector[2] = pos.arr[2];
}

void toASN1SCC(const Eigen::Quaternionf& quat, asn1SccOrientation& orient)
{
	// In a Eigen Quaternion the coefficients are stored internally in the
	// following order: [x, y, z, w], so we use the same order here.
	// See: https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#ad30f4da9a2c0c8dd95520ee8a6d14ef6
	orient.arr[0] = quat.x();
	orient.arr[1] = quat.y();
	orient.arr[2] = quat.z();
	orient.arr[3] = quat.w();
}

void fromASN1SCC(const asn1SccOrientation& orient, Eigen::Quaternionf& quat)
{
	// In a Eigen Quaternion the coefficients are stored internally in the
	// following order: [x, y, z, w], so we use the same order here.
	// See: https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#ad30f4da9a2c0c8dd95520ee8a6d14ef6
	quat.x() = orient.arr[0];
	quat.y() = orient.arr[1];
	quat.z() = orient.arr[2];
	quat.w() = orient.arr[3];
}

void toASN1SCC(const Eigen::Matrix3f& eigen_rot_matrix, asn1SccOrientation& orient)
{
	Eigen::Quaternionf quat(eigen_rot_matrix);
	toASN1SCC(quat, orient);
}

void fromASN1SCC(const asn1SccOrientation& orient, Eigen::Matrix3f& eigen_rot_matrix)
{
	Eigen::Quaternionf quat(eigen_rot_matrix);
	fromASN1SCC(orient, quat);
	eigen_rot_matrix = quat.toRotationMatrix();
}

void toASN1SCC(const Eigen::Affine3f& eigen_transform, asn1SccPose& pose)
{
	toASN1SCC(eigen_transform.translation(), pose.pos);
	toASN1SCC(eigen_transform.rotation(), pose.orient);
}

// TODO: Fix this function
// void fromASN1SCC(const Pose& pose, Eigen::Affine3f& eigen_transform)
// {
// 	fromASN1SCC(pose.pos, eigen_transform.translation());
// 	fromASN1SCC(pose.orient, eigen_transform.rotation());
// }
