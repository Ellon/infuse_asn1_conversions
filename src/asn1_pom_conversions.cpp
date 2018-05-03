#include "conversions/asn1_pom_conversions.hpp"
#include "conversions/asn1_conversions.hpp"

using namespace std;

void toASN1SCC(const PositionManager::Pose& pose, Pose_Infuse& asnPose)
{
    asnPose.msgVersion = pose_Infuse_Version;

    toASN1SCC(pose._parent, asnPose.parentFrameId);
    asnPose.parentTime.microseconds = pose._parentTime;
    asnPose.parentTime.usecPerSec = PositionManager::TimeManager::USEC_PER_SEC;
    toASN1SCC(pose._child, asnPose.childFrameId);
    asnPose.childTime.microseconds  = pose._childTime;
    asnPose.childTime.usecPerSec = PositionManager::TimeManager::USEC_PER_SEC;
    
    asnPose.transform.translation.nCount = 3;
    asnPose.transform.translation.arr[0] = pose._tr.transform.translation(0);
    asnPose.transform.translation.arr[1] = pose._tr.transform.translation(1);
    asnPose.transform.translation.arr[2] = pose._tr.transform.translation(2);

    asnPose.transform.orientation.nCount = 4;
    asnPose.transform.orientation.arr[0] = pose._tr.transform.orientation.w();
    asnPose.transform.orientation.arr[1] = pose._tr.transform.orientation.x();
    asnPose.transform.orientation.arr[2] = pose._tr.transform.orientation.y();
    asnPose.transform.orientation.arr[3] = pose._tr.transform.orientation.z();

    asnPose.transform.cov.nCount = 6;
    for(int i = 0; i < 6; i++)
    {
        asnPose.transform.cov.arr[i].nCount = 6;
        for(int j = 0; j < 6; j++)
        {
            asnPose.transform.cov.arr[i].arr[j] = pose._tr.transform.cov(i,j);
        }
    }
}

void fromASN1SCC(const Pose_Infuse& asnPose, PositionManager::Pose& pose)
{
    fromASN1SCC(asnPose.parentFrameId, pose._parent);
    pose._parentTime = asnPose.parentTime.microseconds;
    fromASN1SCC(asnPose.childFrameId, pose._child);
    pose._childTime = asnPose.childTime.microseconds;

    pose._tr.transform.translation(0) = asnPose.transform.translation.arr[0];
    pose._tr.transform.translation(1) = asnPose.transform.translation.arr[1];
    pose._tr.transform.translation(2) = asnPose.transform.translation.arr[2];
    
    pose._tr.transform.orientation.w() = asnPose.transform.orientation.arr[0];
    pose._tr.transform.orientation.x() = asnPose.transform.orientation.arr[1];
    pose._tr.transform.orientation.y() = asnPose.transform.orientation.arr[2];
    pose._tr.transform.orientation.z() = asnPose.transform.orientation.arr[3];
    
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            pose._tr.transform.cov(i,j) = asnPose.transform.cov.arr[i].arr[j];
        }
    }
}
