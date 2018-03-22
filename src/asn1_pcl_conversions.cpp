#include "conversions/asn1_pcl_conversions.hpp"

#include "conversions/asn1_conversions.hpp"

void toASN1SCC(const pcl::uint64_t& pcl_stamp, Time& time)
{
	// pcl::PCLHeader::stamp represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch).
	time.microseconds = (T_Int64)pcl_stamp;
	// This should be a constant.
	// See: https://www.rock-robotics.org/stable/api/base/types/structbase_1_1Time.html#a9ebef61fd3740771e8f7ff888e41c9cd
	time.usecPerSec = 1000000ll;
}

void fromASN1SCC(const Time& time, pcl::uint64_t& pcl_stamp)
{
	pcl_stamp = (pcl::uint64_t)time.microseconds;
}

void toASN1SCC(const pcl::PCLHeader& pcl_header, Header& header)
{
	toASN1SCC(pcl_header.frame_id, header.frameId);
	toASN1SCC(pcl_header.stamp,    header.timeStamp);
}

void fromASN1SCC(const Header& header, pcl::PCLHeader& pcl_header)
{
	fromASN1SCC(header.frameId,   pcl_header.frame_id);
	fromASN1SCC(header.timeStamp, pcl_header.stamp);
}

void toASN1SCC(const pcl::PointXYZ& pcl_pt, Point& pt)
{
	pt.arr[0] = pcl_pt.x;
	pt.arr[1] = pcl_pt.y;
	pt.arr[2] = pcl_pt.z;
	pt.nCount = 3;
}

void fromASN1SCC(const Point& pt, pcl::PointXYZ& pcl_pt)
{
	if(pt.nCount < 3)
		throw std::runtime_error("Invalid Point size ( size < 3)");
	pcl_pt.x = pt.arr[0];
	pcl_pt.y = pt.arr[1];
	pcl_pt.z = pt.arr[2];
}

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const PoseStamped& pose_stamped, PointCloudPoseStamped& cloud)
{
	toASN1SCC(pcl_cloud.header, cloud.header);
	// TODO: Fix this attribution, as ASN1 types are structors with pointers,
	// and trivial attribution may copy the pointers and lead to bugs
	cloud.pose = pose_stamped;
	cloud.pointCloudData.nCount = std::min((T_UInt32)pcl_cloud.points.size(), maxSizePointCloud);
	for(int i = 0; i < cloud.pointCloudData.nCount; i++)
		toASN1SCC(pcl_cloud.points[i], cloud.pointCloudData.arr[i]);
}

void fromASN1SCC(const PointCloudPoseStamped& cloud, pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, PoseStamped& pose_stamped)
{
	fromASN1SCC(cloud.header, pcl_cloud.header);
	// TODO: Fix this attribution, as ASN1 types are structors with pointers,
	// and trivial attribution may copy the pointers and lead to bugs
	pose_stamped = cloud.pose;
	pcl_cloud.points.resize(cloud.pointCloudData.nCount);
	for(unsigned int i = 0; i < pcl_cloud.points.size(); i++)
	{
		fromASN1SCC(cloud.pointCloudData.arr[i], pcl_cloud.points[i]);
	}
}