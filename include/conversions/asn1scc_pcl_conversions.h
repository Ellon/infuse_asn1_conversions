#ifndef _ASN1SCC_PCL_CONVERSIONS_H_
#define _ASN1SCC_PCL_CONVERSIONS_H_

// -----------
// PCL types 
// -----------
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLHeader.h>

#include <Time.h>
#include <Header.h>
#include <Point.h>
#include <PoseStamped.h>
#include <PointCloudPoseStamped.h>


void toASN1SCC(const pcl::uint64_t& pcl_stamp, Time& time)
{
	// pcl::PCLHeader::stamp represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch).
	time.microseconds = (T_Int64)pcl_stamp;
	// This should be a constant.
	// See: https://www.rock-robotics.org/stable/api/base/types/structbase_1_1Time.html#a9ebef61fd3740771e8f7ff888e41c9cd
	time.usecPerSec = 1000000ll;
}

void toASN1SCC(const pcl::PCLHeader& pcl_header, Header& header)
{
	toASN1SCC(pcl_header.frame_id, header.frameId);
	toASN1SCC(pcl_header.stamp,    header.timeStamp);
}

void toASN1SCC(const pcl::PointXYZ& pcl_pt, Point& pt)
{
	pt.arr[0] = pcl_pt.x;
	pt.arr[1] = pcl_pt.y;
	pt.arr[2] = pcl_pt.z;
	pt.nCount = 3;
}

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const PoseStamped& pose_stamped, PointCloudPoseStamped& cloud)
{
	toASN1SCC(pcl_cloud.header, cloud.header);
	cloud.pose = pose_stamped;
	cloud.pointCloudData.nCount = std::min((T_UInt32)pcl_cloud.points.size(), maxSizePointCloud);
	for(unsigned int i = 0; i < cloud.pointCloudData.nCount; i++)
		toASN1SCC(pcl_cloud.points[i], cloud.pointCloudData.arr[i]);
}

#endif // _ASN1SCC_PCL_CONVERSIONS_H_