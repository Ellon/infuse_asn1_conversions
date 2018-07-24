#ifndef DEF_ASN1SCC_POSITIONMANAGER_CONVERSIONS_H
#define DEF_ASN1SCC_POSITIONMANAGER_CONVERSIONS_H

#include <Pose_InFuse.h>
#include <PositionManagerBase.hpp>

void toASN1SCC(const PositionManager::Pose& pose, Pose_InFuse& asnPose);
void fromASN1SCC(const Pose_InFuse& asnPose, PositionManager::Pose& pose);

#endif


