// Ticket: 0021_worldmodel_asset_refactor
// Design: docs/designs/worldmodel-asset-refactor/design.md

#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include <stdexcept>

namespace msd_sim
{

AssetPhysical::AssetPhysical(msd_assets::CollisionGeometry&& geometry,
                             const ReferenceFrame& frame)
  : visualGeometry_{std::move(geometry)},
    collisionHull_{visualGeometry_.getVertices()},
    referenceFrame_{frame}
{
  if (visualGeometry_.getVertexCount() == 0)
  {
    throw std::invalid_argument(
      "Cannot create AssetPhysical with empty geometry");
  }
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

}  // namespace msd_sim
