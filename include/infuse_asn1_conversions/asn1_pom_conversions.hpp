#ifndef DEF_ASN1SCC_POSITIONMANAGER_CONVERSIONS_H
#define DEF_ASN1SCC_POSITIONMANAGER_CONVERSIONS_H

#include <infuse_asn1_types/TransformWithCovariance.h>
#include <infuse_pom_base/PositionManagerBase.hpp>

void toASN1SCC(const PositionManager::Pose& pose, asn1SccTransformWithCovariance& asnPose);
void fromASN1SCC(const asn1SccTransformWithCovariance& asnPose, PositionManager::Pose& pose);

#endif


