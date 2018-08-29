#ifndef __ASN1SCC_OPENCV_CONVERSIONS__
#define __ASN1SCC_OPENCV_CONVERSIONS__

#include <opencv2/opencv.hpp>
#include <iostream>
#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_types/Map.h>

bool fromASN1SCC(const asn1SccMap &demMsg, cv::Mat &demMatrix);
bool toASN1SCC(const cv::Mat &demMatrix, asn1SccMap &demMsg);

#endif
