#ifndef _ASN1SCC_PCL_CONVERSIONS_H_
#define _ASN1SCC_PCL_CONVERSIONS_H_

// -----------
// PCL types 
// -----------
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// -----------
// ASN1 types 
// -----------
#include <infuse_cdff_types/Time.h>
#include <infuse_cdff_types/Point.h>
#include <infuse_cdff_types/PointCloud_InFuse.h>


void toASN1SCC(const pcl::uint64_t& pcl_stamp, Time& time);

void fromASN1SCC(const Time& time, pcl::uint64_t& pcl_stamp);

void toASN1SCC(const pcl::PointXYZ& pcl_pt, Point& pt);

void fromASN1SCC(const Point& pt, pcl::PointXYZ& pcl_pt);

void toASN1SCC(const pcl::PointXYZI& pcl_pt, Point& pt);

void fromASN1SCC(const Point& pt, pcl::PointXYZI& pcl_pt);

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud, PointCloud_InFuse& cloud);

void fromASN1SCC(const PointCloud_InFuse& cloud, pcl::PointCloud<pcl::PointXYZI>& pcl_cloud);

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, PointCloud_InFuse& cloud);


#endif // _ASN1SCC_PCL_CONVERSIONS_H_