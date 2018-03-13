#ifndef __ASN1SCC_OPENCV_CONVERSIONS__
#define __ASN1SCC_OPENCV_CONVERSIONS__

#include <opencv2/opencv.hpp>
#include <iostream>
#include <DEM.h>

using namespace std;

bool fromASN1SCC(const DigitalElevationRaster &demMsg, cv::Mat &demMatrix)
{

    int nbCols = demMsg.nbCols;
    int nbLines = demMsg.nbLines;
    if (nbCols != demMatrix.cols || nbLines != demMatrix.rows)
    {
        cerr << "Wrong dimensions of input Matrix [" << demMatrix.rows << "x" << demMatrix.cols <<"] While expecting [" << nbLines << "x" << nbCols << "]" <<  endl;
        return false;
    }
    for (int i = 0; i < nbCols; i++)
    {
        for (int j = 0; i < nbLines; j++)
        {
            demMatrix.at<float>(j,i) = demMsg.zValue.arr[i + j*nbCols];
        }
    }
    return true;
}

#endif
