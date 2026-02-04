// Ticket: 0027a_expanding_polytope_algorithm (refactoring)
// Extracted from GJK.cpp and EPA.cpp to eliminate duplication

#include "msd-sim/src/Physics/SupportFunction.hpp"
#include <limits>

namespace msd_sim
{
namespace SupportFunction
{

Coordinate support(const ConvexHull& hull, const Coordinate& dir)
{
  const auto& vertices = hull.getVertices();

  double maxDot = -std::numeric_limits<double>::infinity();
  Coordinate furthest{0.0, 0.0, 0.0};

  for (const auto& vertex : vertices)
  {
    double dotProduct = vertex.dot(dir);
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
                            const Eigen::Vector3d& dir)
{
  const ConvexHull& hullA = assetA.getCollisionHull();
  const ConvexHull& hullB = assetB.getCollisionHull();
  const ReferenceFrame& frameA = assetA.getReferenceFrame();
  const ReferenceFrame& frameB = assetB.getReferenceFrame();

  // Transform search direction from world space to local space for asset A
  // (rotation only - direction vectors don't translate)
  Coordinate dirA_local = frameA.globalToLocal(dir);

  // Get support vertex in local space for asset A
  Coordinate supportA_local = support(hullA, dirA_local);

  // Transform support vertex from local space to world space
  // (rotation + translation - positions transform fully)
  Coordinate supportA_world = frameA.localToGlobal(supportA_local);

  // Same process for asset B with negated direction
  Coordinate dirB_local = frameB.globalToLocal(Eigen::Vector3d{-dir});
  Coordinate supportB_local = support(hullB, dirB_local);
  Coordinate supportB_world = frameB.localToGlobal(supportB_local);

  // Return Minkowski difference in world space
  return supportA_world - supportB_world;
}

SupportResult supportMinkowskiWithWitness(const AssetPhysical& assetA,
                                          const AssetPhysical& assetB,
                                          const Eigen::Vector3d& dir)
{
  const ConvexHull& hullA = assetA.getCollisionHull();
  const ConvexHull& hullB = assetB.getCollisionHull();
  const ReferenceFrame& frameA = assetA.getReferenceFrame();
  const ReferenceFrame& frameB = assetB.getReferenceFrame();

  // Transform search direction from world space to local space for asset A
  // (rotation only - direction vectors don't translate)
  Coordinate dirA_local = frameA.globalToLocal(dir);

  // Get support vertex in local space for asset A
  Coordinate supportA_local = support(hullA, dirA_local);

  // Transform support vertex from local space to world space
  // (rotation + translation - positions transform fully)
  Coordinate supportA_world = frameA.localToGlobal(supportA_local);

  // Same process for asset B with negated direction
  Coordinate dirB_local = frameB.globalToLocal(Eigen::Vector3d{-dir});
  Coordinate supportB_local = support(hullB, dirB_local);
  Coordinate supportB_world = frameB.localToGlobal(supportB_local);

  // Return Minkowski difference with witness points
  return SupportResult{supportA_world - supportB_world,  // Minkowski
                       supportA_world,                   // Witness A
                       supportB_world};                  // Witness B
}

}  // namespace SupportFunction
}  // namespace msd_sim
