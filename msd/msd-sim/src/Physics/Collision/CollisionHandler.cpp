// Ticket: 0027a_expanding_polytope_algorithm
// Ticket: 0047_face_contact_manifold_generation
// Ticket: 0055c_friction_direction_fix
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#include "msd-sim/src/Physics/Collision/CollisionHandler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
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
  // Ticket: 0055c_friction_direction_fix

  // The contact normal in EPA convention points from A toward B.
  // The SAT normal points in the direction of minimum overlap.
  // We need it oriented from A toward B (same as EPA convention).
  // supportMinkowski(A, B, d) = support_A(d) - support_B(-d)
  // overlap = supportMink(d) · d > 0 means A extends past B along d.
  // The correction pushes A in the -d direction (toward B side).
  // So the contact normal (from A toward B) is -satNormal.
  Vector3D const contactDir{
    -sat.normal.x(), -sat.normal.y(), -sat.normal.z()};

  double const depth = std::max(sat.depth, 0.0);
  Coordinate const normal{contactDir.x(), contactDir.y(), contactDir.z()};

  // Detect vertex-face geometry and generate multi-point manifold if applicable
  // Ticket: 0055c_friction_direction_fix
  //
  // Extract face vertices from both assets to determine contact geometry.
  // SAT normal points in minimum penetration direction, so:
  // - Reference face vertices from asset B (face perpendicular to -sat.normal)
  // - Incident vertices from asset A (along +sat.normal)
  std::vector<Coordinate> refFaceVerts = extractFaceVertices(assetB, -sat.normal);
  std::vector<Coordinate> incVerts = extractFaceVertices(assetA, sat.normal);

  if (vertexFaceDetector_.isVertexFaceContact(refFaceVerts.size(),
                                               incVerts.size()))
  {
    // Vertex-face contact: generate multi-point manifold
    std::array<ContactPoint, 4> contacts{};

    // Determine which side is the vertex and which is the face
    if (incVerts.size() == 1)
    {
      // A is incident vertex, B is reference face
      size_t count = vertexFaceManifoldGenerator_.generate(
        refFaceVerts, incVerts[0], Vector3D{normal.x(), normal.y(), normal.z()},
        depth, contacts);

      if (count >= 3)
      {
        return CollisionResult{normal, depth, contacts, count};
      }
    }
    else if (refFaceVerts.size() == 1)
    {
      // B is incident vertex, A is reference face (swap roles)
      // Extract reference face from A (along +sat.normal)
      std::vector<Coordinate> refFaceVertsA = extractFaceVertices(assetA, sat.normal);

      size_t count = vertexFaceManifoldGenerator_.generate(
        refFaceVertsA, refFaceVerts[0],
        Vector3D{normal.x(), normal.y(), normal.z()}, depth, contacts);

      if (count >= 3)
      {
        return CollisionResult{normal, depth, contacts, count};
      }
    }
  }

  // Fall back to single-point contact (existing behavior)
  SupportResult const witness =
    support_function::supportMinkowskiWithWitness(assetA, assetB, sat.normal);

  std::array<ContactPoint, 4> contacts{};
  contacts[0] = ContactPoint{witness.witnessA, witness.witnessB, depth};

  return CollisionResult{normal, depth, contacts, 1};
}

std::vector<Coordinate> CollisionHandler::extractFaceVertices(
  const AssetPhysical& asset,
  const Vector3D& normal) const
{
  // Find the face most aligned with the given normal and return its vertices.
  //
  // Ticket: 0055c_friction_direction_fix
  //
  // Strategy:
  // 1. Transform normal to asset's local space
  // 2. Find facet with maximum alignment (normal · facetNormal)
  // 3. Collect unique vertices from that facet
  // 4. Transform vertices to world space

  const auto& hull = asset.getCollisionHull();
  const auto& frame = asset.getReferenceFrame();

  // Transform normal from world space to local space (rotation only)
  Vector3D localNormal = frame.globalToLocalRelative(normal);

  // Find best-matching facet
  double bestAlignment = -std::numeric_limits<double>::infinity();
  const Facet* bestFacet = nullptr;

  for (const auto& facet : hull.getFacets())
  {
    Vector3D facetNormal{facet.normal.x(), facet.normal.y(), facet.normal.z()};
    double alignment = localNormal.dot(facetNormal);

    if (alignment > bestAlignment)
    {
      bestAlignment = alignment;
      bestFacet = &facet;
    }
  }

  // If no face found (shouldn't happen), return empty
  if (!bestFacet)
  {
    return {};
  }

  // Collect unique vertices from the facet
  // ConvexHull stores vertices as a flat vector; facet.vertexIndices are indices
  const auto& hullVertices = hull.getVertices();
  std::unordered_set<size_t> uniqueIndices{bestFacet->vertexIndices.begin(),
                                             bestFacet->vertexIndices.end()};

  std::vector<Coordinate> faceVertices;
  faceVertices.reserve(uniqueIndices.size());

  for (size_t idx : uniqueIndices)
  {
    // Transform from local space to world space
    Coordinate worldVert = frame.localToGlobalAbsolute(hullVertices[idx]);
    faceVertices.push_back(worldVert);
  }

  return faceVertices;
}

}  // namespace msd_sim
