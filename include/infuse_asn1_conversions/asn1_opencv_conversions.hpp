#ifndef __ASN1SCC_OPENCV_CONVERSIONS__
#define __ASN1SCC_OPENCV_CONVERSIONS__

#include <opencv2/opencv.hpp>
#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <iostream>
#include <DEM.h>

bool fromASN1SCC(const DigitalElevationRaster &demMsg, cv::Mat &demMatrix);
bool toASN1SCC(const cv::Mat &demMatrix, DigitalElevationRaster &demMsg);

#endif
