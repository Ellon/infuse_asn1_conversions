#include "infuse_asn1_conversions/asn1_pom_conversions.hpp"
#include "infuse_asn1_conversions/asn1_base_conversions.hpp"

using namespace std;

void toASN1SCC(const PositionManager::Pose& pose, asn1SccTransformWithCovariance& asnPose)
{
    asnPose.metadata.msgVersion = transformWithCovariance_version;
    toASN1SCC(string(""), asnPose.metadata.producerId);

    // Convert metadata 
    toASN1SCC(pose._parent, asnPose.metadata.parentFrameId);
    asnPose.metadata.parentTime.microseconds = pose._parentTime;
    asnPose.metadata.parentTime.usecPerSec = PositionManager::TimeManager::USEC_PER_SEC;
    toASN1SCC(pose._child, asnPose.metadata.childFrameId);
    asnPose.metadata.childTime.microseconds  = pose._childTime;
    asnPose.metadata.childTime.usecPerSec = PositionManager::TimeManager::USEC_PER_SEC;
    
    // Convert data 
    asnPose.data.translation.arr[0] = pose._tr.transform.translation(0);
    asnPose.data.translation.arr[1] = pose._tr.transform.translation(1);
    asnPose.data.translation.arr[2] = pose._tr.transform.translation(2);

    asnPose.data.orientation.arr[0] = pose._tr.transform.orientation.x();
    asnPose.data.orientation.arr[1] = pose._tr.transform.orientation.y();
    asnPose.data.orientation.arr[2] = pose._tr.transform.orientation.z();
    asnPose.data.orientation.arr[3] = pose._tr.transform.orientation.w();

    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            asnPose.data.cov.arr[i].arr[j] = pose._tr.transform.cov(i,j);
        }
    }
}

void fromASN1SCC(const asn1SccTransformWithCovariance& asnPose, PositionManager::Pose& pose)
{
    // Convert metadata 
    fromASN1SCC(asnPose.metadata.parentFrameId, pose._parent);
    pose._parentTime = asnPose.metadata.parentTime.microseconds;
    fromASN1SCC(asnPose.metadata.childFrameId, pose._child);
    pose._childTime = asnPose.metadata.childTime.microseconds;

    // Convert data
    pose._tr.transform.translation(0) = asnPose.data.translation.arr[0];
    pose._tr.transform.translation(1) = asnPose.data.translation.arr[1];
    pose._tr.transform.translation(2) = asnPose.data.translation.arr[2];
    
    pose._tr.transform.orientation.x() = asnPose.data.orientation.arr[0];
    pose._tr.transform.orientation.y() = asnPose.data.orientation.arr[1];
    pose._tr.transform.orientation.z() = asnPose.data.orientation.arr[2];
    pose._tr.transform.orientation.w() = asnPose.data.orientation.arr[3];
    
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            pose._tr.transform.cov(i,j) = asnPose.data.cov.arr[i].arr[j];
        }
    }
}
