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

void toASN1SCC(const pcl::PointXYZI& pcl_pt, Point& pt)
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

void fromASN1SCC(const Point& pt, pcl::PointXYZI& pcl_pt)
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

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud, PointCloud_InFuse& cloud)
{
	// Version
	cloud.msgVersion = pointCloud_Infuse_Version;

	// timeStamp
	toASN1SCC(pcl_cloud.header.stamp, cloud.timeStamp);

	// Set points
	unsigned int pt_count = 0;
	for (size_t i = 0; i < pcl_cloud.points.size(); i++) {
		if (pcl::isFinite(pcl_cloud.points[i]) and pcl_isfinite(pcl_cloud.points[i].intensity)) {
			toASN1SCC(pcl_cloud.points[i], cloud.points.arr[pt_count]);
			cloud.intensity.arr[pt_count] = (T_Int32)pcl_cloud.points[i].intensity;
			pt_count++;
		}
		if (pt_count >= maxSize)
			break;
	}

	// Set number of points
	cloud.points.nCount = (int)pt_count;
	cloud.intensity.nCount = (int)pt_count;

	// Set oganization info
	if (pt_count == pcl_cloud.points.size()) {
		cloud.isOrdered = (pcl_cloud.height > 1);
		cloud.height = pcl_cloud.height;
		cloud.width = pcl_cloud.width;
	} else {
		// we truncated the point cloud (either due to ASN.1 'maxSize' limit
		// or because there were invalid points), so the cloud is not
		// organized anymore.
		cloud.isOrdered = false;
		cloud.height = 1;
		cloud.width = pt_count;
	}

	// No color information
	cloud.isRegistered = false;
	PointCloud_InFuse_colors_Initialize(&cloud.colors);

	// The following information does not depend on the point cloud and should
	// be set from outside this function
	//
	// cloud.sensorId = ?;
	// cloud.frameId = ?;
	// cloud.hasFixedTransform = ?;
	// cloud.pose_robotFrame_sensorFrame = ?;
	// cloud.pose_fixedFrame_robotFrame = ?;
}

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, PointCloud_InFuse& cloud)
{
	// Version
	cloud.msgVersion = pointCloud_Infuse_Version;

	// timeStamp
	toASN1SCC(pcl_cloud.header.stamp, cloud.timeStamp);

	// Set points
	unsigned int pt_count = 0;
	for (size_t i = 0; i < pcl_cloud.points.size(); i++) {
		if (pcl::isFinite(pcl_cloud.points[i])) {
			toASN1SCC(pcl_cloud.points[i], cloud.points.arr[pt_count]);
			pt_count++;
		}
		if (pt_count >= maxSize)
			break;
	}

	// Set number of points
	cloud.points.nCount = (int)pt_count;

	// Set oganization info
	if (pt_count == pcl_cloud.points.size()) {
		cloud.isOrdered = (pcl_cloud.height > 1);
		cloud.height = pcl_cloud.height;
		cloud.width = pcl_cloud.width;
	} else {
		// we truncated the point cloud (either due to ASN.1 'maxSize' limit
		// or because there were invalid points), so the cloud is not
		// organized anymore.
		cloud.isOrdered = false;
		cloud.height = 1;
		cloud.width = pt_count;
	}

	// No intensity information
	PointCloud_InFuse_intensity_Initialize(&cloud.intensity);

	// No color information
	cloud.isRegistered = false;
	PointCloud_InFuse_colors_Initialize(&cloud.colors);

	// The following information does not depend on the point cloud and should
	// be set from outside this function
	//
	// cloud.sensorId = ?;
	// cloud.frameId = ?;
	// cloud.hasFixedTransform = ?;
	// cloud.pose_robotFrame_sensorFrame = ?;
	// cloud.pose_fixedFrame_robotFrame = ?;
}