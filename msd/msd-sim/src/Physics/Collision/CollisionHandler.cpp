// Ticket: 0027a_expanding_polytope_algorithm
// Ticket: 0047_face_contact_manifold_generation
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "msd-sim/src/DataTypes/Vector3D.hpp"
#include "msd-sim/src/Physics/Collision/EPA.hpp"
#include "msd-sim/src/Physics/Collision/GJK.hpp"
#include "msd-sim/src/Physics/SupportFunction.hpp"

namespace msd_sim
{

CollisionHandler::CollisionHandler(double epsilon) : epsilon_{epsilon}
{
}

std::optional<CollisionResult> CollisionHandler::checkCollision(
  const AssetPhysical& assetA,
  const AssetPhysical& assetB,
  bool skipSATValidation) const
{
  // Phase 1: Broad intersection test via GJK
  GJK gjk{assetA, assetB, epsilon_};

  if (!gjk.intersects())
  {
    return std::nullopt;
  }

  // Phase 2: Detailed contact info via EPA
  EPA epa{assetA, assetB, epsilon_};
  CollisionResult result = epa.computeContactInfo(gjk.getSimplex());

  // Phase 3: Validate EPA result using SAT minimum penetration depth
  //
  // Ticket: 0047_face_contact_manifold_generation
  //
  // EPA can produce catastrophically wrong results when shapes are
  // barely touching (zero penetration). At zero penetration, the origin
  // lies on the Minkowski difference boundary. GJK's simplex becomes
  // degenerate, and EPA picks an arbitrary face — often one with a huge
  // depth along the wrong axis (e.g., 50m along x for a 1m cube inside
  // a 100m floor).
  //
  // Fix: Compute the true minimum penetration depth using SAT. For convex
  // polyhedra, the minimum penetration direction is always along one of the
  // face normals. If EPA's depth is wildly inconsistent with SAT, replace
  // EPA's result with a SAT-derived contact.
  //
  // Ticket: 0053d_sat_fallback_cost_reduction
  // Optimization: Skip SAT for persistent contacts where the ContactCache
  // confirms a stable normal from the previous frame. EPA failures occur
  // primarily on NEW contacts (first frame of collision) or when the normal
  // changes significantly. Persistent contacts with stable normals are safe.
  if (skipSATValidation)
  {
    return result;
  }

  SATResult sat = computeSATMinPenetration(assetA, assetB);

  // If EPA result is consistent with SAT, use EPA (it has better manifolds)
  if (result.penetrationDepth <= sat.depth * 10.0 + epsilon_)
  {
    return result;
  }

  // EPA picked the wrong face. Fall back to SAT-derived contact.
  // This handles both zero-penetration (touching) and near-zero cases
  // where EPA's degenerate simplex causes arbitrary face selection.
  return buildSATContact(assetA, assetB, sat);
}

CollisionHandler::SATResult CollisionHandler::computeSATMinPenetration(
  const AssetPhysical& assetA,
  const AssetPhysical& assetB) const
{
  // Compute the minimum penetration depth over all face normals of both
  // hulls using the SAT approach.
  //
  // For a given direction d, the overlap (penetration) is:
  //   overlap(d) = supportMinkowski(A, B, d) · d
  //
  // Where supportMinkowski returns support_A(d) - support_B(-d).
  // The dot product with d gives the projection of the Minkowski difference
  // support point onto d, which equals the overlap of A and B along d.
  //
  // The minimum overlap over all face normals gives the minimum penetration
  // depth. If any overlap ≤ 0, the shapes are separated (should not happen
  // since GJK confirmed intersection).

  const auto& hullA = assetA.getCollisionHull();
  const auto& hullB = assetB.getCollisionHull();
  const auto& frameA = assetA.getReferenceFrame();
  const auto& frameB = assetB.getReferenceFrame();

  double minOverlap = std::numeric_limits<double>::infinity();
  Vector3D bestNormal{0.0, 0.0, 1.0};  // Fallback normal

  // Collect unique face normals from both hulls (in world space)
  auto checkFaceNormals =
    [&](const ConvexHull& hull, const ReferenceFrame& frame)
  {
    // Track normals we've already checked to skip duplicates
    // (Qhull triangulates, so cubes have 12 facets but only 6 unique normals)
    std::vector<Vector3D> checkedNormals;

    for (const auto& facet : hull.getFacets())
    {
      // Transform facet normal from local space to world space (rotation only)
      Vector3D const localNormal{
        facet.normal.x(), facet.normal.y(), facet.normal.z()};
      Vector3D const worldNormal = frame.localToGlobalRelative(localNormal);

      // Skip near-duplicate normals (triangulated faces of the same plane)
      bool isDuplicate = false;
      for (const auto& checked : checkedNormals)
      {
        if ((worldNormal - checked).norm() < 1e-6)
        {
          isDuplicate = true;
          break;
        }
      }
      if (isDuplicate)
      {
        continue;
      }
      checkedNormals.push_back(worldNormal);

      // Compute overlap along this normal
      Coordinate const support =
        support_function::supportMinkowski(assetA, assetB, worldNormal);
      double const overlap = support.dot(
        Coordinate{worldNormal.x(), worldNormal.y(), worldNormal.z()});

      if (overlap < minOverlap)
      {
        minOverlap = overlap;
        bestNormal = worldNormal;
      }
    }
  };

  checkFaceNormals(hullA, frameA);
  checkFaceNormals(hullB, frameB);

  return SATResult{minOverlap, bestNormal};
}

CollisionResult CollisionHandler::buildSATContact(
  const AssetPhysical& assetA,
  const AssetPhysical& assetB,
  const SATResult& sat) const
{
  // Build a CollisionResult using SAT-derived normal and depth.
  // The SAT normal is the true minimum penetration direction; witness
  // points are computed via the support function along that normal.
  //
  // Ticket: 0047_face_contact_manifold_generation

  // The contact normal in EPA convention points from A toward B.
  // The SAT normal points in the direction of minimum overlap.
  // We need it oriented from A toward B (same as EPA convention).
  // supportMinkowski(A, B, d) = support_A(d) - support_B(-d)
  // overlap = supportMink(d) · d > 0 means A extends past B along d.
  // The correction pushes A in the -d direction (toward B side).
  // So the contact normal (from A toward B) is -satNormal.
  Vector3D const contactDir{
    -sat.normal.x(), -sat.normal.y(), -sat.normal.z()};

  SupportResult const witness =
    support_function::supportMinkowskiWithWitness(assetA, assetB, sat.normal);

  double const depth = std::max(sat.depth, 0.0);

  Coordinate const normal{contactDir.x(), contactDir.y(), contactDir.z()};
  std::array<ContactPoint, 4> contacts{};
  contacts[0] = ContactPoint{witness.witnessA, witness.witnessB, depth};

  return CollisionResult{normal, depth, contacts, 1};
}

}  // namespace msd_sim
