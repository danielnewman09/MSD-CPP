// Ticket: 0022_gjk_asset_physical_transform
// Design: docs/designs/0022_gjk_asset_physical_transform/design.md

#ifndef MSD_SIM_PHYSICS_GJK_HPP
#define MSD_SIM_PHYSICS_GJK_HPP

#include <vector>

#include "msd-sim/src/Physics/RigidBody/AssetPhysical.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace msd_sim
{

/**
 * @brief GJK (Gilbert-Johnson-Keerthi) collision detection algorithm.
 *
 * The GJK algorithm efficiently detects collision between convex shapes by
 * iteratively constructing a simplex in Minkowski difference space that
 * attempts to contain the origin.
 *
 * https://computerwebsite.net/writing/gjk
 *
 * Key insight: Two convex shapes A and B intersect if and only if their
 * Minkowski difference (A ⊖ B) contains the origin.
 *
 * This class maintains the algorithm state (simplex and search direction)
 * as it iterates toward a solution. It works with AssetPhysical objects
 * to support collision detection between objects with arbitrary world-space
 * transformations.
 *
 * @see
 * docs/designs/0022_gjk_asset_physical_transform/0022_gjk_asset_physical_transform.puml
 * @ticket 0022_gjk_asset_physical_transform
 */
class GJK
{
public:
  /**
   * @brief Construct a GJK solver for two physical assets with transformations.
   *
   * @param assetA First physical asset (includes collision hull and reference
   * frame)
   * @param assetB Second physical asset (includes collision hull and reference
   * frame)
   * @param epsilon Numerical tolerance for convergence (default: 1e-6)
   */
  GJK(const AssetPhysical& assetA,
      const AssetPhysical& assetB,
      double epsilon = 1e-6);

  /**
   * @brief Test if the two assets intersect in world space.
   *
   * @param maxIterations Maximum iterations before giving up (default: 64)
   * @return true if the assets intersect, false otherwise
   */
  bool intersects(int maxIterations = 64);

  /**
   * @brief Get the terminating simplex for EPA input.
   *
   * @pre intersects() must have been called and returned true.
   *      Behavior is undefined if called before intersects() or
   *      after intersects() returned false.
   *
   * @return Const reference to simplex vertices (4 vertices forming
   *         a tetrahedron in Minkowski space that contains the origin)
   */
  [[nodiscard]] const std::vector<Coordinate>& getSimplex() const { return simplex_; }

private:
  const AssetPhysical& assetA_;
  const AssetPhysical& assetB_;
  double epsilon_;

  std::vector<Coordinate> simplex_;
  Coordinate direction_;

  /**
   * @brief Update simplex and search direction.
   * @return true if origin is contained in simplex (collision detected)
   */
  bool updateSimplex();

  /**
   * @brief Handle line simplex case (2 points).
   */
  bool handleLine();

  /**
   * @brief Handle triangle simplex case (3 points).
   */
  bool handleTriangle();

  /**
   * @brief Handle tetrahedron simplex case (4 points).
   */
  bool handleTetrahedron();

  /**
   * @brief Check if point is in the same direction as direction vector.
   */
  static bool sameDirection(const Coordinate& direction, const Coordinate& ao);
};

/**
 * @brief Convenience function to test intersection using GJK with AssetPhysical
 * objects.
 *
 * @param assetA First physical asset
 * @param assetB Second physical asset
 * @param epsilon Numerical tolerance for convergence (default: 1e-6)
 * @param maxIterations Maximum iterations before giving up (default: 64)
 * @return true if the assets intersect in world space, false otherwise
 */
bool gjkIntersects(const AssetPhysical& assetA,
                   const AssetPhysical& assetB,
                   double epsilon = 1e-6,
                   int maxIterations = 64);

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_GJK_HPP