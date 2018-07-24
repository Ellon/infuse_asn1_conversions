#ifndef _ASN1SCC_ROS_CONVERSIONS_H_
#define _ASN1SCC_ROS_CONVERSIONS_H_

// -----------
// ROS types 
// -----------
#include <ros/time.h>

// -----------
// ASN1 types 
// -----------
#include <infuse_asn1_types/Time.h>


void toASN1SCC(const ros::Time& ros_stamp, Time& time);

void fromASN1SCC(const Time& time, ros::Time& ros_stamp);


#endif // _ASN1SCC_ROS_CONVERSIONS_H_