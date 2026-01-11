// Ticket: 0021_worldmodel_asset_refactor
// Design: docs/designs/worldmodel-asset-refactor/design.md

#include "msd-sim/src/Physics/RigidBody/AssetEnvironment.hpp"

namespace msd_sim
{

AssetEnvironment::AssetEnvironment(msd_assets::CollisionGeometry&& geometry,
                                   const ReferenceFrame& frame)
  : AssetPhysical{std::move(geometry), frame}
{
}

// Deprecated factory method (kept for backward compatibility)
std::shared_ptr<AssetEnvironment> AssetEnvironment::create(
  std::shared_ptr<msd_assets::CollisionGeometry> geometry,
  const ReferenceFrame& frame,
  bool /* computeHull */)
{
  if (!geometry)
  {
    throw std::invalid_argument(
      "Cannot create AssetEnvironment with null geometry");
  }

  // Copy geometry and create via public constructor
  msd_assets::CollisionGeometry geomCopy{*geometry};
  return std::make_shared<AssetEnvironment>(std::move(geomCopy), frame);
}

}  // namespace msd_sim
