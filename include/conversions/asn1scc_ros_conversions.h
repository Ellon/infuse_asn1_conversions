#ifndef _ASN1SCC_ROS_CONVERSIONS_H_
#define _ASN1SCC_ROS_CONVERSIONS_H_

// -----------
// ROS types 
// -----------
#include <ros/time.h>


void toASN1SCC(const ros::Time& ros_stamp, Time& time)
{
	time.microseconds = (T_Int64)(ros_stamp.toNSec() / 1000ull);
	// This should be a constant.
	// See: https://www.rock-robotics.org/stable/api/base/types/structbase_1_1Time.html#a9ebef61fd3740771e8f7ff888e41c9cd
	time.usecPerSec = 1000000ll;
}

#endif // _ASN1SCC_ROS_CONVERSIONS_H_