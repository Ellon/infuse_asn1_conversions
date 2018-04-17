#ifndef DEF_ASN1SCC_POSITIONMANAGER_CONVERSIONS_H
#define DEF_ASN1SCC_POSITIONMANAGER_CONVERSIONS_H

#include <PoseDoubleStamped.h>
#include <PositionManagerBase.hpp>

void toASN1SCC(const PositionManager::Pose& pose, PoseDoubleStamped & asnPose);
void fromASN1SCC(const PoseDoubleStamped& asnPose, PositionManager::Pose& pose);

#endif


