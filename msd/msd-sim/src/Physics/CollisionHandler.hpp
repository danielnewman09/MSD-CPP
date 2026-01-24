// Ticket: 0027a_expanding_polytope_algorithm
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#ifndef MSD_SIM_PHYSICS_COLLISION_HANDLER_HPP
#define MSD_SIM_PHYSICS_COLLISION_HANDLER_HPP

#include <optional>
#include "msd-sim/src/Physics/CollisionResult.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"

namespace msd_sim
{

/**
 * @brief Orchestrates collision detection algorithms (GJK/EPA).
 *
 * Provides a unified interface for collision detection that:
 * - Runs GJK to detect intersection
 * - If collision detected, runs EPA to compute contact info
 * - Returns std::optional to indicate collision presence
 *
 * This abstraction allows future enhancements (broadphase,
 * continuous collision detection, etc.) without changing callers.
 *
 * @see docs/designs/0027a_expanding_polytope_algorithm/0027a_expanding_polytope_algorithm.puml
 * @ticket 0027a_expanding_polytope_algorithm
 */
class CollisionHandler
{
public:
  /**
   * @brief Construct handler with specified tolerance.
   *
   * @param epsilon Numerical tolerance for GJK/EPA (default: 1e-6)
   */
  explicit CollisionHandler(double epsilon = 1e-6);

  /**
   * @brief Check for collision between two physical assets.
   *
   * Runs GJK to detect intersection. If collision detected,
   * runs EPA to compute penetration depth, contact normal,
   * and contact point.
   *
   * @param assetA First physical asset
   * @param assetB Second physical asset
   * @return std::nullopt if no collision, CollisionResult if collision
   */
  std::optional<CollisionResult> checkCollision(
      const AssetPhysical& assetA,
      const AssetPhysical& assetB) const;

  CollisionHandler(const CollisionHandler&) = default;
  CollisionHandler(CollisionHandler&&) noexcept = default;
  CollisionHandler& operator=(const CollisionHandler&) = default;
  CollisionHandler& operator=(CollisionHandler&&) noexcept = default;
  ~CollisionHandler() = default;

private:
  double epsilon_;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_COLLISION_HANDLER_HPP
