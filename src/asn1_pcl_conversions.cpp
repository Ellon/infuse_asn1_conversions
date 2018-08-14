#include "infuse_asn1_conversions/asn1_pcl_conversions.hpp"

#include "infuse_asn1_conversions/asn1_base_conversions.hpp"

void toASN1SCC(const pcl::uint64_t& pcl_stamp, asn1SccTime& time)
{
	// pcl::PCLHeader::stamp represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch).
	time.microseconds = (asn1SccT_Int64)pcl_stamp;
	// This should be a constant.
	// See: https://www.rock-robotics.org/stable/api/base/types/structbase_1_1Time.html#a9ebef61fd3740771e8f7ff888e41c9cd
	time.usecPerSec = 1000000ll;
}

void fromASN1SCC(const asn1SccTime& time, pcl::uint64_t& pcl_stamp)
{
	pcl_stamp = (pcl::uint64_t)time.microseconds;
}

void toASN1SCC(const pcl::PointXYZ& pcl_pt, asn1SccPoint& pt)
{
	pt.arr[0] = pcl_pt.x;
	pt.arr[1] = pcl_pt.y;
	pt.arr[2] = pcl_pt.z;
}

void toASN1SCC(const pcl::PointXYZI& pcl_pt, asn1SccPoint& pt)
{
	pt.arr[0] = pcl_pt.x;
	pt.arr[1] = pcl_pt.y;
	pt.arr[2] = pcl_pt.z;
}

void fromASN1SCC(const asn1SccPoint& pt, pcl::PointXYZ& pcl_pt)
{
	pcl_pt.x = pt.arr[0];
	pcl_pt.y = pt.arr[1];
	pcl_pt.z = pt.arr[2];
}

void fromASN1SCC(const asn1SccPoint& pt, pcl::PointXYZI& pcl_pt)
{
	pcl_pt.x = pt.arr[0];
	pcl_pt.y = pt.arr[1];
	pcl_pt.z = pt.arr[2];
}

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud, asn1SccPointcloud& cloud)
{
	// Version
	cloud.metadata.msgVersion = pointCloud_Version;

	// timeStamp
	toASN1SCC(pcl_cloud.header.stamp, cloud.metadata.timeStamp);

	// frameId
	toASN1SCC(pcl_cloud.header.frame_id, cloud.metadata.frameId);

	// Set points
	unsigned int pt_count = 0;
	for (size_t i = 0; i < pcl_cloud.points.size(); i++) {
		if (pcl::isFinite(pcl_cloud.points[i]) and pcl_isfinite(pcl_cloud.points[i].intensity)) {
			toASN1SCC(pcl_cloud.points[i], cloud.data.points.arr[pt_count]);
			cloud.data.intensity.arr[pt_count] = (asn1SccT_Int32)pcl_cloud.points[i].intensity;
			pt_count++;
		}
		if (pt_count >= maxPointcloudSize)
			break;
	}

	// Set number of points
	cloud.data.points.nCount = (int)pt_count;
	cloud.data.intensity.nCount = (int)pt_count;

	// Set oganization info
	if (pt_count == pcl_cloud.points.size()) {
		cloud.metadata.isOrdered = (pcl_cloud.height > 1);
		cloud.metadata.height = pcl_cloud.height;
		cloud.metadata.width = pcl_cloud.width;
	} else {
		// we truncated the point cloud (either due to ASN.1 'maxSize' limit
		// or because there were invalid points), so the cloud is not
		// organized anymore.
		cloud.metadata.isOrdered = false;
		cloud.metadata.height = 1;
		cloud.metadata.width = pt_count;
	}

	// No color information
	cloud.metadata.isRegistered = false;
	asn1SccPointCloud_Data_colors_Initialize(&cloud.data.colors);

	// The following information does not depend on the point cloud and should
	// be set from outside this function
	//
	// cloud.metadata.sensorId = ?;
	// cloud.metadata.hasFixedTransform = ?;
	// cloud.metadata.pose_robotFrame_sensorFrame = ?;
	// cloud.metadata.pose_fixedFrame_robotFrame = ?;
}

void fromASN1SCC(const asn1SccPointcloud& cloud, pcl::PointCloud<pcl::PointXYZI>& pcl_cloud)
{
	// timeStamp
	fromASN1SCC(cloud.metadata.timeStamp, pcl_cloud.header.stamp);

	// frameId
	fromASN1SCC(cloud.metadata.frameId, pcl_cloud.header.frame_id);

	// Set points
	pcl_cloud.points.resize(std::min(cloud.data.points.nCount, cloud.data.intensity.nCount));
	for (size_t i = 0; i < pcl_cloud.points.size(); i++) {
		fromASN1SCC(cloud.data.points.arr[i], pcl_cloud.points[i]);
		pcl_cloud.points[i].intensity = (float) cloud.data.intensity.arr[i];
	}

	// Set oganization info
	if (cloud.metadata.isOrdered) {
		pcl_cloud.height = cloud.metadata.height;
		pcl_cloud.width = cloud.metadata.width;
	} else {
		pcl_cloud.height = 1;
		pcl_cloud.width = pcl_cloud.points.size();
	}

	// No place to store color information in the pcl cloud
	// cloud.metadata.isRegistered = false;
	// asn1SccPointcloud_colors_Initialize(&cloud.colors);

	// The following information does not depend on the point cloud and should
	// be set from outside this function
	//
	// cloud.metadata.sensorId = ?;
	// cloud.metadata.hasFixedTransform = ?;
	// cloud.metadata.pose_robotFrame_sensorFrame = ?;
	// cloud.metadata.pose_fixedFrame_robotFrame = ?;
}

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, asn1SccPointcloud& cloud)
{
	// Version
	cloud.metadata.msgVersion = pointCloud_Version;

	// timeStamp
	toASN1SCC(pcl_cloud.header.stamp, cloud.metadata.timeStamp);

	// Set points
	unsigned int pt_count = 0;
	for (size_t i = 0; i < pcl_cloud.points.size(); i++) {
		if (pcl::isFinite(pcl_cloud.points[i])) {
			toASN1SCC(pcl_cloud.points[i], cloud.data.points.arr[pt_count]);
			pt_count++;
		}
		if (pt_count >= maxPointcloudSize)
			break;
	}

	// Set number of points
	cloud.data.points.nCount = (int)pt_count;

	// Set oganization info
	if (pt_count == pcl_cloud.points.size()) {
		cloud.metadata.isOrdered = (pcl_cloud.height > 1);
		cloud.metadata.height = pcl_cloud.height;
		cloud.metadata.width = pcl_cloud.width;
	} else {
		// we truncated the point cloud (either due to ASN.1 'maxSize' limit
		// or because there were invalid points), so the cloud is not
		// organized anymore.
		cloud.metadata.isOrdered = false;
		cloud.metadata.height = 1;
		cloud.metadata.width = pt_count;
	}

	// No intensity information
	asn1SccPointCloud_Data_intensity_Initialize(&cloud.data.intensity);

	// No color information
	cloud.metadata.isRegistered = false;
	asn1SccPointCloud_Data_colors_Initialize(&cloud.data.colors);

	// The following information does not depend on the point cloud and should
	// be set from outside this function
	//
	// cloud.metadata.sensorId = ?;
	// cloud.metadata.frameId = ?;
	// cloud.metadata.hasFixedTransform = ?;
	// cloud.metadata.pose_robotFrame_sensorFrame = ?;
	// cloud.metadata.pose_fixedFrame_robotFrame = ?;
}