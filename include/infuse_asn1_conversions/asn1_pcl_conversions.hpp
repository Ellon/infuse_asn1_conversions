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
#include <infuse_asn1_types/Time.h>
#include <infuse_asn1_types/Point.h>
#include <infuse_asn1_types/Pointcloud.h>


void toASN1SCC(const pcl::uint64_t& pcl_stamp, asn1SccTime& time);

void fromASN1SCC(const asn1SccTime& time, pcl::uint64_t& pcl_stamp);

void toASN1SCC(const pcl::PointXYZ& pcl_pt, asn1SccPoint& pt);

void fromASN1SCC(const asn1SccPoint& pt, pcl::PointXYZ& pcl_pt);

void toASN1SCC(const pcl::PointXYZI& pcl_pt, asn1SccPoint& pt);

void fromASN1SCC(const asn1SccPoint& pt, pcl::PointXYZI& pcl_pt);

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud, asn1SccPointcloud& cloud);

void fromASN1SCC(const asn1SccPointcloud& cloud, pcl::PointCloud<pcl::PointXYZI>& pcl_cloud);

void toASN1SCC(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, asn1SccPointcloud& cloud);


#endif // _ASN1SCC_PCL_CONVERSIONS_H_