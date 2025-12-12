#ifndef MSD_SIM_PHYSICS_GJK_HPP
#define MSD_SIM_PHYSICS_GJK_HPP

#include <vector>
#include "msd-sim/src/Physics/ConvexHull.hpp"

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
 * as it iterates toward a solution.
 */
class GJK
{
public:
  /**
   * @brief Construct a GJK solver for two convex hulls.
   *
   * @param hullA First convex hull
   * @param hullB Second convex hull
   * @param epsilon Numerical tolerance for convergence (default: 1e-6)
   */
  GJK(const ConvexHull& hullA, const ConvexHull& hullB, float epsilon = 1e-6f);

  /**
   * @brief Test if the two hulls intersect.
   *
   * @param maxIterations Maximum iterations before giving up (default: 64)
   * @return true if the hulls intersect, false otherwise
   */
  bool intersects(int maxIterations = 64);

private:
  const ConvexHull& hullA_;
  const ConvexHull& hullB_;
  float epsilon_;

  std::vector<Coordinate> simplex_;
  Coordinate direction_;

  /**
   * @brief Support function: find vertex furthest in given direction.
   */
  Coordinate support(const ConvexHull& hull, const Coordinate& dir) const;

  /**
   * @brief Minkowski difference support function.
   */
  Coordinate supportMinkowski(const Coordinate& dir) const;

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
 * @brief Convenience function to test intersection using GJK.
 *
 * @param hullA First convex hull
 * @param hullB Second convex hull
 * @param epsilon Numerical tolerance for convergence (default: 1e-6)
 * @param maxIterations Maximum iterations before giving up (default: 64)
 * @return true if the hulls intersect, false otherwise
 */
bool gjkIntersects(const ConvexHull& hullA,
                   const ConvexHull& hullB,
                   float epsilon = 1e-6f,
                   int maxIterations = 64);

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_GJK_HPP