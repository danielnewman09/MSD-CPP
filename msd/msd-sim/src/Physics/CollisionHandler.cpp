// Ticket: 0027a_expanding_polytope_algorithm
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#include "msd-sim/src/Physics/CollisionHandler.hpp"
#include "msd-sim/src/Physics/EPA.hpp"
#include "msd-sim/src/Physics/GJK.hpp"

namespace msd_sim
{

CollisionHandler::CollisionHandler(double epsilon) : epsilon_{epsilon} {}

std::optional<CollisionResult> CollisionHandler::checkCollision(
    const AssetPhysical& assetA,
    const AssetPhysical& assetB) const
{
  // Phase 1: Broad intersection test via GJK
  GJK gjk{assetA, assetB, epsilon_};

  if (!gjk.intersects())
  {
    // No collision - return empty optional
    return std::nullopt;
  }

  // Phase 2: Detailed contact info via EPA
  // Only reached when GJK confirms intersection
  EPA epa{assetA, assetB, epsilon_};
  CollisionResult result = epa.computeContactInfo(gjk.getSimplex());

  return result;
}

}  // namespace msd_sim
