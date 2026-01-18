#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"

namespace msd_sim
{

AssetPhysical::AssetPhysical(uint32_t assetId,
                             uint32_t instanceId,
                             ConvexHull& hull,
                             const ReferenceFrame& frame)
  : referenceAssetId_{assetId},
    instanceId_{instanceId},
    collisionHull_{hull},
    referenceFrame_(frame)
{
}

const ConvexHull& AssetPhysical::getCollisionHull() const
{
  return collisionHull_;
}

const ReferenceFrame& AssetPhysical::getReferenceFrame() const
{
  return referenceFrame_;
}

ReferenceFrame& AssetPhysical::getReferenceFrame()
{
  return referenceFrame_;
}


uint32_t AssetPhysical::getAssetId() const
{
  return referenceAssetId_;
}


uint32_t AssetPhysical::getInstanceId() const
{
  return instanceId_;
}


}  // namespace msd_sim
