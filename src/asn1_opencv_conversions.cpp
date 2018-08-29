#include "infuse_asn1_conversions/asn1_opencv_conversions.hpp"

bool fromASN1SCC(const asn1SccMap &demMsg, cv::Mat &demMatrix)
{

    int nbCols = demMsg.data.cols;
    int nbLines = demMsg.data.rows;
    if (nbCols != demMatrix.cols || nbLines != demMatrix.rows)
    {
        std::cerr << "Wrong dimensions of input Matrix [" << demMatrix.rows << "x" << demMatrix.cols <<"] While expecting [" << nbLines << "x" << nbCols << "]" <<  std::endl;
        std::cerr << "Resizing the matrix" << std::endl;
        demMatrix.create(nbLines,nbCols,CV_32FC1);
    }

    for (int i = 0 ; i < nbLines; i++)
    {
        memcpy(demMatrix.data + i * demMatrix.step, &demMsg.data.data.arr[i*demMatrix.step], demMatrix.step);
    }

    return true;
}

bool toASN1SCC(const cv::Mat &demMatrix, asn1SccMap &demMsg)
{

    /* DEM dimensions */

    demMsg.data.cols = demMatrix.cols;
    demMsg.data.rows = demMatrix.rows;

    /*Filling the data*/

    int i;
    for (i = 0; i < demMatrix.rows; i++)
    {
        memcpy(&demMsg.data.data.arr[i*demMatrix.step],demMatrix.data + i * demMatrix.step, demMatrix.step);
    }

    /* Don't forget the count */
    demMsg.data.data.nCount = demMsg.data.rows * demMatrix.step;
    return true;

}

