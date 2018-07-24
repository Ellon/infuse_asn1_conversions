#include "infuse_asn1_conversions/asn1_ros_conversions.hpp"

void toASN1SCC(const ros::Time& ros_stamp, Time& time)
{
	time.microseconds = (T_Int64)(ros_stamp.toNSec() / 1000ull);
	// This should be a constant.
	// See: https://www.rock-robotics.org/stable/api/base/types/structbase_1_1Time.html#a9ebef61fd3740771e8f7ff888e41c9cd
	time.usecPerSec = 1000000ll;
}

void fromASN1SCC(const Time& time, ros::Time& ros_stamp)
{
	// ROS Time stores its time in nano-secons, so we're losing precision
	// here.
	ros_stamp.fromNSec((uint64_t)time.microseconds * 1000ull);  // Convert from us to ns
}
