// Ticket: 0027a_expanding_polytope_algorithm
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include "msd-sim/src/Physics/Collision/EPA.hpp"
#include "msd-sim/src/Physics/SupportFunction.hpp"

namespace msd_sim
{

EPA::EPA(const AssetPhysical& assetA,
         const AssetPhysical& assetB,
         double epsilon)
  : assetA_{assetA}, assetB_{assetB}, epsilon_{epsilon}
{
}

CollisionResult EPA::computeContactInfo(const std::vector<Coordinate>& simplex,
                                        int maxIterations)
{
  // Validate simplex size
  if (simplex.size() < 4)
  {
    // Edge case: GJK detected collision before building full tetrahedron
    // This can happen when direction becomes near-zero during GJK iteration
    // Build a minimal tetrahedron by adding support points with witness
    // tracking

    // Re-query support for existing simplex vertices to get proper witness
    // points We query in the direction of each vertex and adjust witnesses to
    // maintain the invariant: witnessA - witnessB = minkowski (original point)
    vertices_.reserve(4);
    for (const auto& point : simplex)
    {
      double const norm = point.norm();
      msd_sim::Vector3D const dir =
        (norm < epsilon_)
          ? msd_sim::Vector3D{1, 0, 0}
          : msd_sim::Vector3D{
              point.x() / norm, point.y() / norm, point.z() / norm};
      SupportResult const support =
        support_function::supportMinkowskiWithWitness(assetA_, assetB_, dir);

      // Adjust witnesses to maintain invariant: witnessA - witnessB = point
      // offset = point - support.minkowski (difference between original and
      // queried) witnessA' = witnessA + offset/2, witnessB' = witnessB -
      // offset/2 Then witnessA' - witnessB' = witnessA - witnessB + offset =
      // support.minkowski + offset = point
      Coordinate const offset = point - support.minkowski;
      Coordinate const witnessA = support.witnessA + offset * 0.5;
      Coordinate const witnessB = support.witnessB - offset * 0.5;

      vertices_.emplace_back(point, witnessA, witnessB);
    }

    // Add support points in cardinal directions until we have 4 vertices
    std::vector<msd_sim::Vector3D> const directions = {
      msd_sim::Vector3D{1.0, 0.0, 0.0},
      msd_sim::Vector3D{0.0, 1.0, 0.0},
      msd_sim::Vector3D{0.0, 0.0, 1.0},
      msd_sim::Vector3D{-1.0, 0.0, 0.0},
      msd_sim::Vector3D{0.0, -1.0, 0.0},
      msd_sim::Vector3D{0.0, 0.0, -1.0}};

    for (const auto& dir : directions)
    {
      if (vertices_.size() >= 4)
      {
        break;
      }

      SupportResult const support =
        support_function::supportMinkowskiWithWitness(assetA_, assetB_, dir);

      // Only add if not duplicate (within epsilon)
      bool isDuplicate = false;
      for (const auto& existing : vertices_)
      {
        if ((support.minkowski - existing.point).norm() < epsilon_)
        {
          isDuplicate = true;
          break;
        }
      }

      if (!isDuplicate)
      {
        vertices_.emplace_back(
          support.minkowski, support.witnessA, support.witnessB);
      }
    }

    // If still < 4 vertices, we have a degenerate case
    if (vertices_.size() < 4)
    {
      throw std::runtime_error(
        "EPA cannot build tetrahedron: degenerate collision geometry");
    }
  }
  else if (simplex.size() > 4)
  {
    throw std::invalid_argument(
      "EPA simplex cannot have more than 4 vertices (must be tetrahedron)");
  }
  else
  {
    // Normal case: initialize polytope with GJK terminating simplex
    // GJK simplex only has Minkowski points, not witness points.
    // Re-query support function with witness tracking and adjust to maintain
    // invariant.
    vertices_.reserve(4);
    for (const auto& point : simplex)
    {
      double const norm = point.norm();
      msd_sim::Vector3D const dir =
        (norm < epsilon_)
          ? msd_sim::Vector3D{1, 0, 0}
          : msd_sim::Vector3D{
              point.x() / norm, point.y() / norm, point.z() / norm};
      SupportResult const support =
        support_function::supportMinkowskiWithWitness(assetA_, assetB_, dir);

      // Adjust witnesses to maintain invariant: witnessA - witnessB = point
      Coordinate const offset = point - support.minkowski;
      Coordinate const witnessA = support.witnessA + offset * 0.5;
      Coordinate const witnessB = support.witnessB - offset * 0.5;

      vertices_.emplace_back(point, witnessA, witnessB);
    }
  }

  // Create 4 initial faces from tetrahedron
  // Vertices: A=0, B=1, C=2, D=3
  // Faces (outward-facing normals):
  // ABC (away from D), ACD (away from B), ADB (away from C), BDC (away from A)
  addFace(0, 1, 2);  // ABC
  addFace(0, 2, 3);  // ACD
  addFace(0, 3, 1);  // ADB
  addFace(1, 3, 2);  // BDC

  // Expand polytope until convergence
  // Ticket: 0048 — graceful degradation: use best result when max iterations
  // reached instead of throwing. This handles near-degenerate geometry where
  // EPA stalls (e.g., identical micro-rotations from PositionCorrector).
  expandPolytope(maxIterations);

  // Extract contact information from closest face (best approximation if not
  // fully converged)
  size_t const closestFaceIndex = findClosestFace();
  const Facet& closestFace = faces_[closestFaceIndex];

  // Extract contact manifold (Ticket: 0029_contact_manifold_generation)
  std::array<ContactPoint, 4> contacts;
  size_t const contactCount =
    extractContactManifold(closestFaceIndex, contacts);

  return CollisionResult{
    closestFace.normal, closestFace.offset, contacts, contactCount};
}

bool EPA::expandPolytope(int maxIterations)
{
  for (int iteration = 0; iteration < maxIterations; ++iteration)
  {
    // Find closest face to origin
    size_t const closestFaceIndex = findClosestFace();
    const Facet& closestFace = faces_[closestFaceIndex];

    // Query support point in direction of closest face normal with witness
    // tracking
    SupportResult const support = support_function::supportMinkowskiWithWitness(
      assetA_, assetB_, closestFace.normal);

    // Convergence check: if new point is within tolerance of face distance,
    // we've found the closest point on the Minkowski boundary
    double const distanceToNewPoint = support.minkowski.dot(closestFace.normal);
    if (distanceToNewPoint - closestFace.offset < epsilon_)
    {
      // Converged - closest face found
      return true;
    }

    // Expansion: remove visible faces, build horizon, add new faces
    std::vector<EPAEdge> const horizonEdges =
      buildHorizonEdges(support.minkowski);

    // Add new vertex to polytope with witness tracking
    size_t const newVertexIndex = vertices_.size();
    vertices_.emplace_back(
      support.minkowski, support.witnessA, support.witnessB);

    // Create new faces connecting new vertex to horizon edges
    for (const auto& edge : horizonEdges)
    {
      addFace(edge.v0, edge.v1, newVertexIndex);
    }
  }

  // Max iterations reached without convergence
  return false;
}

size_t EPA::findClosestFace() const
{
  double minDistance = std::numeric_limits<double>::infinity();
  size_t closestIndex = 0;

  for (size_t i = 0; i < faces_.size(); ++i)
  {
    if (faces_[i].offset < minDistance)
    {
      minDistance = faces_[i].offset;
      closestIndex = i;
    }
  }

  return closestIndex;
}

bool EPA::isVisible(const Facet& face, const Coordinate& point) const
{
  // Point is visible from face if it's on the positive side of the face plane
  return face.normal.dot(point - vertices_[face.vertexIndices[0]].point) >
         epsilon_;
}

std::vector<EPA::EPAEdge> EPA::buildHorizonEdges(const Coordinate& newVertex)
{
  std::vector<EPAEdge> horizon;
  std::vector<size_t> visibleFaceIndices;

  // Identify visible faces
  for (size_t i = 0; i < faces_.size(); ++i)
  {
    if (isVisible(faces_[i], newVertex))
    {
      visibleFaceIndices.push_back(i);
    }
  }

  // Extract edges from visible faces
  std::vector<EPAEdge> edgeCandidates;
  for (size_t const idx : visibleFaceIndices)
  {
    const auto& f = faces_[idx];
    edgeCandidates.emplace_back(f.vertexIndices[0], f.vertexIndices[1]);
    edgeCandidates.emplace_back(f.vertexIndices[1], f.vertexIndices[2]);
    edgeCandidates.emplace_back(f.vertexIndices[2], f.vertexIndices[0]);
  }

  // Horizon edges appear exactly once (not shared by two visible faces)
  for (const auto& edge : edgeCandidates)
  {
    auto count = std::count(edgeCandidates.begin(), edgeCandidates.end(), edge);
    if (count == 1)
    {
      horizon.push_back(edge);
    }
  }

  // Remove visible faces (erase-remove idiom)
  // Mark faces for deletion by setting distance to infinity
  for (size_t const idx : visibleFaceIndices)
  {
    faces_[idx].offset = std::numeric_limits<double>::infinity();
  }

  // Remove marked faces
  std::erase_if(faces_,
                [](const Facet& face) { return std::isinf(face.offset); });


  return horizon;
}

void EPA::addFace(size_t v0, size_t v1, size_t v2)
{
  const Coordinate& a = vertices_[v0].point;
  const Coordinate& b = vertices_[v1].point;
  const Coordinate& c = vertices_[v2].point;

  // Compute normal via cross product
  Coordinate const ab = b - a;
  Coordinate const ac = c - a;
  Coordinate normal = ab.cross(ac);

  // Normalize
  double const normalLength = normal.norm();
  if (normalLength < epsilon_)
  {
    // Degenerate face (coplanar vertices) - skip
    return;
  }
  normal = normal / normalLength;

  // Ensure normal points away from origin (outward)
  Coordinate const centroid = (a + b + c) / 3.0;
  if (normal.dot(centroid) < 0.0)
  {
    normal = -normal;
    std::swap(v1, v2);  // Flip winding order
  }

  // Distance from origin to face plane
  double const distance = normal.dot(a);

  faces_.emplace_back(v0, v1, v2, normal, distance);
}

Coordinate EPA::computeContactPoint(const Facet& face)
{
  // The closest point on the face to the origin is the projection along the
  // normal closestPoint = origin + normal * distance = normal * offset
  return face.normal * face.offset;
}

namespace
{
// Sutherland-Hodgman polygon clipping against a half-space
// Keeps points on the negative side of the plane (planeNormal · (p -
// planePoint) <= 0)
std::vector<Coordinate> clipPolygonAgainstPlane(
  const std::vector<Coordinate>& polygon,
  const Coordinate& planePoint,
  const msd_sim::Vector3D& planeNormal,
  double epsilon)
{
  if (polygon.empty())
  {
    return {};
  }

  std::vector<Coordinate> output;
  output.reserve(polygon.size() + 1);

  for (size_t i = 0; i < polygon.size(); ++i)
  {
    const Coordinate& current = polygon[i];
    const Coordinate& next = polygon[(i + 1) % polygon.size()];

    double const currentDist = planeNormal.dot(current - planePoint);
    double const nextDist = planeNormal.dot(next - planePoint);

    bool const currentInside = currentDist <= epsilon;
    bool const nextInside = nextDist <= epsilon;

    if (currentInside)
    {
      output.push_back(current);
    }

    // If edge crosses the plane, compute intersection
    if (currentInside != nextInside)
    {
      double const t = currentDist / (currentDist - nextDist);
      Coordinate const intersection = current + (next - current) * t;
      output.push_back(intersection);
    }
  }

  return output;
}

// Build a convex polygon from coplanar facets with correct winding order
// Vertices are sorted counter-clockwise when viewed from faceNormal direction
std::vector<Coordinate> buildPolygonFromFacets(
  const std::vector<std::reference_wrapper<const Facet>>& facets,
  const std::vector<Coordinate>& hullVertices,
  const ReferenceFrame& frame,
  const msd_sim::Vector3D& faceNormal)
{
  // 1. Collect unique vertices (use a set to deduplicate)
  std::set<size_t> uniqueIndices;
  for (const Facet& facet : facets)
  {
    for (size_t const idx : facet.vertexIndices)
    {
      uniqueIndices.insert(idx);
    }
  }

  // 2. Transform vertices to world space
  std::vector<Coordinate> vertices;
  vertices.reserve(uniqueIndices.size());
  for (size_t const idx : uniqueIndices)
  {
    vertices.push_back(frame.localToGlobalAbsolute(hullVertices[idx]));
  }

  // If only 1-2 vertices, return as-is (degenerate case)
  if (vertices.size() < 3)
  {
    return vertices;
  }

  // 3. Compute centroid
  Coordinate centroid{0, 0, 0};
  for (const auto& v : vertices)
  {
    centroid += v;
  }
  centroid /= static_cast<double>(vertices.size());

  // 4. Build a local 2D basis on the face plane
  // Choose an arbitrary vector not parallel to faceNormal
  Coordinate const arbitrary = (std::abs(faceNormal.x()) < 0.9)
                                 ? Coordinate{1, 0, 0}
                                 : Coordinate{0, 1, 0};
  Coordinate basisU = faceNormal.cross(arbitrary).normalized();
  Coordinate basisV = faceNormal.cross(basisU).normalized();

  // 5. Sort vertices by angle around centroid (counter-clockwise)
  std::ranges::sort(
    vertices,

    [&](const Coordinate& a, const Coordinate& b)
    {
      Coordinate const da = a - centroid;
      Coordinate const db = b - centroid;
      double const angleA = std::atan2(da.dot(basisV), da.dot(basisU));
      double const angleB = std::atan2(db.dot(basisV), db.dot(basisU));
      return angleA < angleB;
    });

  return vertices;
}
// Ticket: 0040c_edge_contact_manifold
// Compute the closest pair of points between two line segments in 3D.
// Standard algorithm from Ericson, "Real-Time Collision Detection", Section
// 5.1.9.
std::pair<Coordinate, Coordinate> closestPointsBetweenSegments(
  const Coordinate& p1, const Coordinate& q1,
  const Coordinate& p2, const Coordinate& q2)
{
  Coordinate const d1 = q1 - p1;  // Direction of segment 1
  Coordinate const d2 = q2 - p2;  // Direction of segment 2
  Coordinate const r = p1 - p2;

  double const a = d1.dot(d1);  // Squared length of segment 1
  double const e = d2.dot(d2);  // Squared length of segment 2
  double const f = d2.dot(r);

  double s = 0.0;
  double t = 0.0;

  constexpr double kEpsilon = 1e-10;

  if (a <= kEpsilon && e <= kEpsilon)
  {
    // Both segments degenerate to points
    return {p1, p2};
  }

  if (a <= kEpsilon)
  {
    // Segment 1 degenerates to a point
    s = 0.0;
    t = std::clamp(f / e, 0.0, 1.0);
  }
  else
  {
    double const c = d1.dot(r);
    if (e <= kEpsilon)
    {
      // Segment 2 degenerates to a point
      t = 0.0;
      s = std::clamp(-c / a, 0.0, 1.0);
    }
    else
    {
      // General non-degenerate case
      double const b = d1.dot(d2);
      double const denom = a * e - b * b;  // Always >= 0

      // If segments not parallel, compute closest point on line 1 to line 2
      if (std::abs(denom) > kEpsilon)
      {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      }
      else
      {
        s = 0.0;  // Parallel segments: pick arbitrary s
      }

      // Compute t from s
      t = (b * s + f) / e;

      // Clamp t and recompute s if needed
      if (t < 0.0)
      {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      }
      else if (t > 1.0)
      {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }

  Coordinate const closest1 = p1 + d1 * s;
  Coordinate const closest2 = p2 + d2 * t;
  return {closest1, closest2};
}

}  // namespace

size_t EPA::extractContactManifold(size_t faceIndex,
                                   std::array<ContactPoint, 4>& contacts) const
{
  // Ticket: 0029_contact_manifold_generation
  // Use face clipping (Sutherland-Hodgman) to generate contact manifold

  const Facet& epaFace = faces_[faceIndex];
  const auto& normal = epaFace.normal;

  // Transform EPA normal from world space to each hull's local space
  // before querying aligned facets (hull normals are stored in local space)
  // Ticket: 0039e — fixes single-contact-point bug for rotated objects
  // Ticket: 0041 — use explicit Relative API (rotation-only)
  msd_sim::Vector3D const normalLocalA =
    assetA_.getReferenceFrame().globalToLocalRelative(normal);
  // Note: -normal is an Eigen expression, must wrap in Coordinate for template deduction
  msd_sim::Vector3D const normalLocalB =
    assetB_.getReferenceFrame().globalToLocalRelative(Coordinate{-normal});

  auto facesA = assetA_.getCollisionHull().getFacetsAlignedWith(normalLocalA);
  auto facesB = assetB_.getCollisionHull().getFacetsAlignedWith(normalLocalB);

  double const alignA = std::abs(normalLocalA.dot(facesA[0].get().normal));
  double const alignB = std::abs(normalLocalB.dot(facesB[0].get().normal));

  // Reference face = more aligned with normal (provides clipping planes)
  // Incident face = less aligned (gets clipped)
  // Track which asset is reference using a bool flag
  bool const refIsA = (alignA > alignB);

  const AssetPhysical& refAsset = refIsA ? assetA_ : assetB_;
  const AssetPhysical& incAsset = refIsA ? assetB_ : assetA_;
  auto& refFaces = refIsA ? facesA : facesB;
  auto& incFaces = refIsA ? facesB : facesA;

  // Reference face normal in world space
  const auto& refFrame = refAsset.getReferenceFrame();
  msd_sim::Vector3D const refNormalWorld =
    refFrame.localToGlobalRelative(refFaces[0].get().normal);

  // Build incident polygon from coplanar facets (properly ordered)
  const auto& incFrame = incAsset.getReferenceFrame();
  msd_sim::Vector3D const incNormalWorld =
    incFrame.localToGlobalRelative(incFaces[0].get().normal);
  std::vector<Coordinate> incidentPoly =
    buildPolygonFromFacets(incFaces,
                           incAsset.getCollisionHull().getVertices(),
                           incFrame,
                           incNormalWorld);

  // Build reference polygon from coplanar facets (properly ordered)
  std::vector<Coordinate> refVerts =
    buildPolygonFromFacets(refFaces,
                           refAsset.getCollisionHull().getVertices(),
                           refFrame,
                           refNormalWorld);

  // Handle degenerate cases — attempt vertex-face, then edge contact generation
  // Ticket: 0055c_friction_direction_fix — vertex-face manifold generation
  // Ticket: 0040c_edge_contact_manifold — edge contact generation
  // Ticket: 0040a — fallback uses EPA offset as per-contact depth
  if (refVerts.size() < 3 || incidentPoly.size() < 3)
  {
    // Try vertex-face manifold generation first
    if (vertexFaceDetector_.isVertexFaceContact(refVerts.size(), incidentPoly.size()))
    {
      size_t const vertexFaceContactCount =
        generateVertexFaceManifold(epaFace, refVerts, incidentPoly, refIsA, contacts);
      if (vertexFaceContactCount >= 3)
      {
        return vertexFaceContactCount;
      }
    }

    // Fall back to edge contact generation
    size_t const edgeContactCount =
      generateEdgeContacts(epaFace, contacts);
    if (edgeContactCount >= 2)
    {
      return edgeContactCount;
    }

    // Final fallback: single EPA centroid point
    Coordinate const contactPoint = epaFace.normal * epaFace.offset;
    contacts[0] = ContactPoint{contactPoint, contactPoint, epaFace.offset};
    return 1;
  }

  // Clip incident polygon against each edge plane of the reference face
  // Each side plane is perpendicular to the face, pointing inward
  for (size_t i = 0; i < refVerts.size(); ++i)
  {
    if (incidentPoly.empty())
    {
      break;
    }

    const Coordinate& edgeStart = refVerts[i];
    const Coordinate& edgeEnd = refVerts[(i + 1) % refVerts.size()];

    // Side plane normal: perpendicular to edge and reference face normal,
    // pointing inward (edgeDir × normal points left of edge = inward for CCW
    // winding)
    Coordinate const edgeDir = (edgeEnd - edgeStart).normalized();
    msd_sim::Vector3D const sidePlaneNormal =
      edgeDir.cross(refNormalWorld).normalized();

    incidentPoly = clipPolygonAgainstPlane(
      incidentPoly, edgeStart, sidePlaneNormal, epsilon_);
  }

  // Keep only points at or below the reference face plane
  std::vector<Coordinate> finalPoints;
  double const refPlaneD = refNormalWorld.dot(refVerts[0]);
  for (const auto& point : incidentPoly)
  {
    double const dist = refNormalWorld.dot(point) - refPlaneD;
    if (dist <= epsilon_)
    {
      finalPoints.push_back(point);
    }
  }

  // If no points survived clipping, fall back to EPA centroid contact
  // Ticket: 0040a — fallback uses EPA offset as per-contact depth
  if (finalPoints.empty())
  {
    Coordinate const contactPoint = epaFace.normal * epaFace.offset;
    contacts[0] = ContactPoint{contactPoint, contactPoint, epaFace.offset};
    return 1;
  }

  // Limit to 4 contact points
  size_t const count = std::min(finalPoints.size(), static_cast<size_t>(4));

  // Build contact pairs: project incident points onto reference plane
  // Ticket: 0040a — compute per-contact penetration depth
  for (size_t i = 0; i < count; ++i)
  {
    const Coordinate& incPoint = finalPoints[i];
    double const dist = refNormalWorld.dot(incPoint) - refPlaneD;
    Coordinate const refPoint = incPoint - refNormalWorld * dist;

    // Per-contact depth: distance from incident point to reference plane.
    // dist < 0 means the point is below the reference face (penetrating),
    // so -dist gives a positive penetration depth value.
    double const pointDepth = std::max(-dist, 0.0);

    // Assign based on which asset owns the reference face
    if (refIsA)
    {
      contacts[i] = ContactPoint{refPoint, incPoint, pointDepth};
    }
    else
    {
      contacts[i] = ContactPoint{incPoint, refPoint, pointDepth};
    }
  }

  return count;
}

// Ticket: 0040c_edge_contact_manifold
// Design: docs/designs/0040c-edge-contact-manifold/design.md
size_t EPA::generateEdgeContacts(
  const Facet& epaFace,
  std::array<ContactPoint, 4>& contacts) const
{
  const auto& normal = epaFace.normal;

  // 1. Compute witness points from EPA face via barycentric interpolation
  const auto& v0 = vertices_[epaFace.vertexIndices[0]];
  const auto& v1 = vertices_[epaFace.vertexIndices[1]];
  const auto& v2 = vertices_[epaFace.vertexIndices[2]];
  Coordinate const witnessA =
    (v0.witnessA + v1.witnessA + v2.witnessA) / 3.0;
  Coordinate const witnessB =
    (v0.witnessB + v1.witnessB + v2.witnessB) / 3.0;

  // 2. Transform witness points to local space for edge query
  const auto& frameA = assetA_.getReferenceFrame();
  const auto& frameB = assetB_.getReferenceFrame();
  Coordinate const localWitnessA = frameA.globalToLocalAbsolute(witnessA);
  Coordinate const localWitnessB = frameB.globalToLocalAbsolute(witnessB);

  // 3. Find closest edge on each hull
  auto edgeA = assetA_.getCollisionHull().findClosestEdge(localWitnessA);
  auto edgeB = assetB_.getCollisionHull().findClosestEdge(localWitnessB);

  // 4. Transform edge endpoints to world space
  Coordinate const edgeA_start = frameA.localToGlobalAbsolute(edgeA.start);
  Coordinate const edgeA_end = frameA.localToGlobalAbsolute(edgeA.end);
  Coordinate const edgeB_start = frameB.localToGlobalAbsolute(edgeB.start);
  Coordinate const edgeB_end = frameB.localToGlobalAbsolute(edgeB.end);

  // 5. Compute closest points between two line segments
  auto [closestOnA, closestOnB] =
    closestPointsBetweenSegments(edgeA_start, edgeA_end,
                                 edgeB_start, edgeB_end);

  // 6. Generate 2 contact points
  // Use the edge endpoints projected onto the contact plane
  // Point 1: closest point between the edges (midpoint)
  Coordinate const midpoint1 = (closestOnA + closestOnB) * 0.5;

  // Point 2: offset along the edge direction to provide geometric extent
  Coordinate const edgeDirA = edgeA_end - edgeA_start;
  Coordinate const edgeDirB = edgeB_end - edgeB_start;
  double const edgeLenA = edgeDirA.norm();
  double const edgeLenB = edgeDirB.norm();

  // Choose the shorter edge length for offset (conservative)
  double const halfExtent = std::min(edgeLenA, edgeLenB) * 0.5;

  // 7. Validate geometric extent
  if (halfExtent < epsilon_)
  {
    return 0;  // Degenerate edge, fall back to single point
  }

  // Edge direction: use longer edge
  Coordinate const edgeDir = (edgeLenA >= edgeLenB)
                               ? edgeDirA / edgeLenA
                               : edgeDirB / edgeLenB;

  // Project edge direction onto contact plane (remove component along normal)
  // IMPORTANT: Construct explicit Vector3D for direction vector operations
  // to avoid the Coordinate overload of globalToLocal which applies
  // translation
  msd_sim::Vector3D const normalVec{normal.x(), normal.y(), normal.z()};
  double const normalComponent = edgeDir.dot(normal);
  Coordinate const edgeDirInPlane =
    (edgeDir - normal * normalComponent).normalized();

  // Check that edge direction in plane is not degenerate
  if (edgeDirInPlane.norm() < epsilon_)
  {
    return 0;  // Edge parallel to normal, degenerate
  }

  // Generate two contact points offset from the closest point
  Coordinate const offset = edgeDirInPlane * halfExtent;

  Coordinate const cp1_mid = midpoint1 + offset;
  Coordinate const cp2_mid = midpoint1 - offset;

  // Project each contact point onto both surfaces along the normal
  // pointA = point projected onto hull A's surface
  // pointB = point projected onto hull B's surface
  double const depth = epaFace.offset;

  contacts[0] = ContactPoint{
    cp1_mid + normal * (depth * 0.5),
    cp1_mid - normal * (depth * 0.5),
    depth};
  contacts[1] = ContactPoint{
    cp2_mid + normal * (depth * 0.5),
    cp2_mid - normal * (depth * 0.5),
    depth};

  return 2;
}

size_t EPA::generateVertexFaceManifold(
  const Facet& epaFace,
  const std::vector<Coordinate>& refVerts,
  const std::vector<Coordinate>& incVerts,
  bool refIsA,
  std::array<ContactPoint, 4>& contacts) const
{
  // Ticket: 0055c_friction_direction_fix
  // Design: docs/designs/0055c_friction_direction_fix/design.md
  //
  // Generate multi-point contact manifold for vertex-face geometry to eliminate
  // energy injection from single-point friction contacts.
  //
  // Algorithm:
  // 1. Identify reference face (>= 3 verts) and incident vertex (1 vert)
  // 2. Use VertexFaceManifoldGenerator to project face verts onto contact plane
  // 3. Build ContactPoint pairs with uniform EPA depth (Option A from design)

  // Determine which side is the face (>= 3 verts) and which is the vertex (1 vert)
  const std::vector<Coordinate>* faceVerts = nullptr;
  Coordinate incidentVertex{};
  bool faceIsA = false;

  if (refVerts.size() >= 3 && incVerts.size() == 1)
  {
    // Reference is face, incident is vertex
    faceVerts = &refVerts;
    incidentVertex = incVerts[0];
    faceIsA = refIsA;
  }
  else if (incVerts.size() >= 3 && refVerts.size() == 1)
  {
    // Incident is face, reference is vertex
    faceVerts = &incVerts;
    incidentVertex = refVerts[0];
    faceIsA = !refIsA;
  }
  else
  {
    // Not vertex-face geometry, should not be called
    return 0;
  }

  // Contact normal in EPA coordinates (points from A toward B)
  Vector3D const contactNormal{epaFace.normal.x(), epaFace.normal.y(), epaFace.normal.z()};

  // Generate manifold using uniform EPA depth (Option A)
  size_t const contactCount = vertexFaceManifoldGenerator_.generate(
    *faceVerts,
    incidentVertex,
    contactNormal,
    epaFace.offset,
    contacts);

  // Adjust contact point assignment based on which asset owns the face
  // EPA convention: contacts[i].pointA is on asset A, contacts[i].pointB is on asset B
  if (!faceIsA)
  {
    // Face is on asset B, vertex is on asset A
    // Generator puts face points in pointA, vertex in pointB
    // Need to swap so vertex (on A) is in pointA, face (on B) is in pointB
    for (size_t i = 0; i < contactCount; ++i)
    {
      std::swap(contacts[i].pointA, contacts[i].pointB);
    }
  }

  return contactCount;
}

}  // namespace msd_sim
