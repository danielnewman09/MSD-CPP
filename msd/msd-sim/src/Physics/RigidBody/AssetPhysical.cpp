#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"

namespace msd_sim
{

AssetPhysical::AssetPhysical(uint32_t assetId,
                             uint32_t instanceId,
                             ConvexHull& hull,
                             ReferenceFrame frame)
  : referenceAssetId_{assetId},
    instanceId_{instanceId},
    collisionHull_{hull},
    referenceFrame_{std::move(frame)}
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

msd_transfer::AssetPhysicalStaticRecord AssetPhysical::toStaticRecord(
  bool isEnvironment) const
{
  msd_transfer::AssetPhysicalStaticRecord record;
  record.body_id = instanceId_;
  record.asset_id = referenceAssetId_;
  record.is_environment = isEnvironment ? 1u : 0u;
  return record;
}

}  // namespace msd_sim
