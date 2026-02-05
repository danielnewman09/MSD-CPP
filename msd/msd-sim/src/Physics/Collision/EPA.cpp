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
      Eigen::Vector3d const dir =
        (norm < epsilon_)
          ? Eigen::Vector3d{1, 0, 0}
          : Eigen::Vector3d{
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
    std::vector<Eigen::Vector3d> const directions = {
      Eigen::Vector3d{1.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, 1.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, 1.0},
      Eigen::Vector3d{-1.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, -1.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, -1.0}};

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
      Eigen::Vector3d const dir =
        (norm < epsilon_)
          ? Eigen::Vector3d{1, 0, 0}
          : Eigen::Vector3d{
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
  if (!expandPolytope(maxIterations))
  {
    throw std::runtime_error("EPA failed to converge within max iterations");
  }

  // Extract contact information from closest face
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
  const Eigen::Vector3d& planeNormal,
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
  const Eigen::Vector3d& faceNormal)
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
    vertices.push_back(frame.localToGlobal(hullVertices[idx]));
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
}  // namespace

size_t EPA::extractContactManifold(size_t faceIndex,
                                   std::array<ContactPoint, 4>& contacts) const
{
  // Ticket: 0029_contact_manifold_generation
  // Use face clipping (Sutherland-Hodgman) to generate contact manifold

  const Facet& epaFace = faces_[faceIndex];
  const auto& normal = epaFace.normal;

  // Find all coplanar faces on each hull aligned with the collision normal
  auto facesA = assetA_.getCollisionHull().getFacetsAlignedWith(normal);
  auto facesB = assetB_.getCollisionHull().getFacetsAlignedWith(-normal);

  double const alignA = std::abs(normal.dot(facesA[0].get().normal));
  double const alignB = std::abs(normal.dot(facesB[0].get().normal));

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
  Eigen::Vector3d const refNormalWorld =
    refFrame.localToGlobal(Eigen::Vector3d{refFaces[0].get().normal.x(),
                                           refFaces[0].get().normal.y(),
                                           refFaces[0].get().normal.z()});

  // Build incident polygon from coplanar facets (properly ordered)
  const auto& incFrame = incAsset.getReferenceFrame();
  Eigen::Vector3d const incNormalWorld =
    incFrame.localToGlobal(Eigen::Vector3d{incFaces[0].get().normal.x(),
                                           incFaces[0].get().normal.y(),
                                           incFaces[0].get().normal.z()});
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

  // Handle degenerate cases
  if (refVerts.size() < 3 || incidentPoly.size() < 3)
  {
    Coordinate const contactPoint = epaFace.normal * epaFace.offset;
    contacts[0] = ContactPoint{contactPoint, contactPoint};
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
    Eigen::Vector3d const sidePlaneNormal =
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
  if (finalPoints.empty())
  {
    Coordinate const contactPoint = epaFace.normal * epaFace.offset;
    contacts[0] = ContactPoint{contactPoint, contactPoint};
    return 1;
  }

  // Limit to 4 contact points
  size_t const count = std::min(finalPoints.size(), static_cast<size_t>(4));

  // Build contact pairs: project incident points onto reference plane
  for (size_t i = 0; i < count; ++i)
  {
    const Coordinate& incPoint = finalPoints[i];
    double const dist = refNormalWorld.dot(incPoint) - refPlaneD;
    Coordinate const refPoint = incPoint - refNormalWorld * dist;

    // Assign based on which asset owns the reference face
    if (refIsA)
    {
      contacts[i] = ContactPoint{refPoint, incPoint};
    }
    else
    {
      contacts[i] = ContactPoint{incPoint, refPoint};
    }
  }

  return count;
}


}  // namespace msd_sim
