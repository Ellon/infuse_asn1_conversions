#ifndef _ASN1SCC_PCL_CONVERSIONS_H_
#define _ASN1SCC_PCL_CONVERSIONS_H_

// -----------
// PCL types 
// -----------
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLHeader.h>

// -----------
// ASN1 types 
// -----------
#include <Time.h>
#include <Header.h>
#include <Point.h>
#include <PoseStamped.h>
#include <PointCloudPoseStamped.h>


void toASN1SCC(const pcl::uint64_t& pcl_stamp, Time& time);

void toASN1SCC(const pcl::PCLHeader& pcl_header, Header& header);

void toASN1SCC(const pcl::PointXYZ& pcl_pt, Point& pt);

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const PoseStamped& pose_stamped, PointCloudPoseStamped& cloud);

#endif // _ASN1SCC_PCL_CONVERSIONS_H_