#include "conversions/asn1_ros_conversions.hpp"

void toASN1SCC(const ros::Time& ros_stamp, Time& time)
{
	time.microseconds = (T_Int64)(ros_stamp.toNSec() / 1000ull);
	// This should be a constant.
	// See: https://www.rock-robotics.org/stable/api/base/types/structbase_1_1Time.html#a9ebef61fd3740771e8f7ff888e41c9cd
	time.usecPerSec = 1000000ll;
}
