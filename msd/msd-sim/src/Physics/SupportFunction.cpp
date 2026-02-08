// Ticket: 0027a_expanding_polytope_algorithm (refactoring)
// Extracted from GJK.cpp and EPA.cpp to eliminate duplication

#include "msd-sim/src/Physics/SupportFunction.hpp"
#include <limits>


namespace msd_sim::support_function
{

Coordinate support(const ConvexHull& hull, const Coordinate& dir)
{
  const auto& vertices = hull.getVertices();

  double maxDot = -std::numeric_limits<double>::infinity();
  Coordinate furthest{0.0, 0.0, 0.0};

  for (const auto& vertex : vertices)
  {
    double const dotProduct = vertex.dot(dir);
    if (dotProduct > maxDot)
    {
      maxDot = dotProduct;
      furthest = vertex;
    }
  }

  return furthest;
}

Coordinate supportMinkowski(const AssetPhysical& assetA,
                            const AssetPhysical& assetB,
                            const msd_sim::Vector3D& dir)
{
  const ConvexHull& hullA = assetA.getCollisionHull();
  const ConvexHull& hullB = assetB.getCollisionHull();
  const ReferenceFrame& frameA = assetA.getReferenceFrame();
  const ReferenceFrame& frameB = assetB.getReferenceFrame();

  // Transform search direction from world space to local space for asset A
  // (rotation only - direction vectors don't translate)
  Coordinate const dirALocal = frameA.globalToLocalRelative(dir);

  // Get support vertex in local space for asset A
  Coordinate const supportALocal = support(hullA, dirALocal);

  // Transform support vertex from local space to world space
  // (rotation + translation - positions transform fully)
  Coordinate const supportAWorld = frameA.localToGlobalAbsolute(supportALocal);

  // Same process for asset B with negated direction
  // Note: -dir is an Eigen expression, must wrap in Vector3D for template deduction
  Coordinate const dirBLocal = frameB.globalToLocalRelative(msd_sim::Vector3D{-dir});
  Coordinate const supportBLocal = support(hullB, dirBLocal);
  Coordinate const supportBWorld = frameB.localToGlobalAbsolute(supportBLocal);

  // Return Minkowski difference in world space
  return supportAWorld - supportBWorld;
}

SupportResult supportMinkowskiWithWitness(const AssetPhysical& assetA,
                                          const AssetPhysical& assetB,
                                          const msd_sim::Vector3D& dir)
{
  const ConvexHull& hullA = assetA.getCollisionHull();
  const ConvexHull& hullB = assetB.getCollisionHull();
  const ReferenceFrame& frameA = assetA.getReferenceFrame();
  const ReferenceFrame& frameB = assetB.getReferenceFrame();

  // Transform search direction from world space to local space for asset A
  // (rotation only - direction vectors don't translate)
  Coordinate const dirALocal = frameA.globalToLocalRelative(dir);

  // Get support vertex in local space for asset A
  Coordinate const supportALocal = support(hullA, dirALocal);

  // Transform support vertex from local space to world space
  // (rotation + translation - positions transform fully)
  Coordinate const supportAWorld = frameA.localToGlobalAbsolute(supportALocal);

  // Same process for asset B with negated direction
  // Note: -dir is an Eigen expression, must wrap in Vector3D for template deduction
  Coordinate const dirBLocal = frameB.globalToLocalRelative(msd_sim::Vector3D{-dir});
  Coordinate const supportBLocal = support(hullB, dirBLocal);
  Coordinate const supportBWorld = frameB.localToGlobalAbsolute(supportBLocal);

  // Return Minkowski difference with witness points
  return SupportResult{supportAWorld - supportBWorld,  // Minkowski
                       supportAWorld,                  // Witness A
                       supportBWorld};                 // Witness B
}

}  // namespace msd_sim::support_function
