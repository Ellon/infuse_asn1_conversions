#ifndef DEF_ASN1SCC_POSITIONMANAGER_CONVERSIONS_H
#define DEF_ASN1SCC_POSITIONMANAGER_CONVERSIONS_H

#include <Pose_Infuse.h>
#include <PositionManagerBase.hpp>

void toASN1SCC(const PositionManager::Pose& pose, Pose_Infuse& asnPose);
void fromASN1SCC(const Pose_Infuse& asnPose, PositionManager::Pose& pose);

#endif


