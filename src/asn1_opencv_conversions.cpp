#include "infuse_asn1_conversions/asn1_opencv_conversions.hpp"

bool fromASN1SCC(const DigitalElevationRaster &demMsg, cv::Mat &demMatrix)
{

    int nbCols = demMsg.nbCols;
    int nbLines = demMsg.nbLines;
    if (nbCols != demMatrix.cols || nbLines != demMatrix.rows)
    {
        std::cerr << "Wrong dimensions of input Matrix [" << demMatrix.rows << "x" << demMatrix.cols <<"] While expecting [" << nbLines << "x" << nbCols << "]" <<  std::endl;
        std::cerr << "Resizing the matrix" << std::endl;
        demMatrix.create(nbLines,nbCols,CV_32FC1);
    }
    for (int i = 0; i < nbCols; i++)
    {
        for (int j = 0; j < nbLines; j++)
        {
            demMatrix.at<float>(j,i) = demMsg.zValue.arr[i + j*nbCols];
        }
    }
    return true;
}

bool toASN1SCC(const cv::Mat &demMatrix, DigitalElevationRaster &demMsg)
{

    /* DEM dimensions */

    demMsg.nbCols = demMatrix.cols;
    demMsg.nbLines = demMatrix.rows;

    /*Filling the data*/

    int i,j;
    for (i = 0; i < demMsg.nbCols; i++)
    {
        for (j = 0; j < demMsg.nbLines; j++)
        {
            demMsg.zValue.arr[i+j*demMsg.nbCols] = demMatrix.at<float>(j,i);
        }
    }

    /* Don't forget the count */
    demMsg.zValue.nCount = demMsg.nbCols*demMsg.nbLines;
    return true;

}

