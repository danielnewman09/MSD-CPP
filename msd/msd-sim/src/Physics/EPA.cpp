// Ticket: 0027a_expanding_polytope_algorithm
// Design: docs/designs/0027a_expanding_polytope_algorithm/design.md

#include "msd-sim/src/Physics/EPA.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "msd-sim/src/Physics/SupportFunction.hpp"

namespace msd_sim
{

EPA::EPA(const AssetPhysical& assetA,
         const AssetPhysical& assetB,
         double epsilon)
  : assetA_{assetA}, assetB_{assetB}, epsilon_{epsilon}, vertices_{}, faces_{}
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
    // Build a minimal tetrahedron by adding support points
    vertices_ = simplex;

    // Add support points in cardinal directions until we have 4 vertices
    std::vector<CoordinateRate> directions = {CoordinateRate{1.0, 0.0, 0.0},
                                              CoordinateRate{0.0, 1.0, 0.0},
                                              CoordinateRate{0.0, 0.0, 1.0},
                                              CoordinateRate{-1.0, 0.0, 0.0},
                                              CoordinateRate{0.0, -1.0, 0.0},
                                              CoordinateRate{0.0, 0.0, -1.0}};

    for (const auto& dir : directions)
    {
      if (vertices_.size() >= 4)
        break;

      Coordinate newVertex =
        SupportFunction::supportMinkowski(assetA_, assetB_, dir);

      // Only add if not duplicate (within epsilon)
      bool isDuplicate = false;
      for (const auto& existing : vertices_)
      {
        if ((newVertex - existing).norm() < epsilon_)
        {
          isDuplicate = true;
          break;
        }
      }

      if (!isDuplicate)
      {
        vertices_.push_back(newVertex);
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
    vertices_ = simplex;
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
  size_t closestFaceIndex = findClosestFace();
  const Facet& closestFace = faces_[closestFaceIndex];

  CollisionResult result;
  result.normal = closestFace.normal;
  result.penetrationDepth = closestFace.offset;
  result.contactPoint = computeContactPoint(closestFace);

  return result;
}

bool EPA::expandPolytope(int maxIterations)
{
  for (int iteration = 0; iteration < maxIterations; ++iteration)
  {
    // Find closest face to origin
    size_t closestFaceIndex = findClosestFace();
    const Facet& closestFace = faces_[closestFaceIndex];

    // Query support point in direction of closest face normal
    Coordinate newPoint =
      SupportFunction::supportMinkowski(assetA_, assetB_, closestFace.normal);

    // Convergence check: if new point is within tolerance of face distance,
    // we've found the closest point on the Minkowski boundary
    double distanceToNewPoint = newPoint.dot(closestFace.normal);
    if (distanceToNewPoint - closestFace.offset < epsilon_)
    {
      // Converged - closest face found
      return true;
    }

    // Expansion: remove visible faces, build horizon, add new faces
    std::vector<EPAEdge> horizonEdges = buildHorizonEdges(newPoint);

    // Add new vertex to polytope
    size_t newVertexIndex = vertices_.size();
    vertices_.push_back(newPoint);

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
  return face.normal.dot(point - vertices_[face.vertexIndices[0]]) > epsilon_;
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
  for (size_t idx : visibleFaceIndices)
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
  for (size_t idx : visibleFaceIndices)
  {
    faces_[idx].offset = std::numeric_limits<double>::infinity();
  }

  // Remove marked faces
  faces_.erase(
    std::remove_if(faces_.begin(),
                   faces_.end(),
                   [](const Facet& face) { return std::isinf(face.offset); }),
    faces_.end());

  return horizon;
}

void EPA::addFace(size_t v0, size_t v1, size_t v2)
{
  const Coordinate& a = vertices_[v0];
  const Coordinate& b = vertices_[v1];
  const Coordinate& c = vertices_[v2];

  // Compute normal via cross product
  Coordinate ab = b - a;
  Coordinate ac = c - a;
  Coordinate normal = ab.cross(ac);

  // Normalize
  double normalLength = normal.norm();
  if (normalLength < epsilon_)
  {
    // Degenerate face (coplanar vertices) - skip
    return;
  }
  normal = normal / normalLength;

  // Ensure normal points away from origin (outward)
  Coordinate centroid = (a + b + c) / 3.0;
  if (normal.dot(centroid) < 0.0)
  {
    normal = -normal;
    std::swap(v1, v2);  // Flip winding order
  }

  // Distance from origin to face plane
  double distance = normal.dot(a);

  faces_.emplace_back(v0, v1, v2, normal, distance);
}

Coordinate EPA::computeContactPoint(const Facet& face) const
{
  // Barycentric centroid (equal weights)
  const Coordinate& a = vertices_[face.vertexIndices[0]];
  const Coordinate& b = vertices_[face.vertexIndices[1]];
  const Coordinate& c = vertices_[face.vertexIndices[2]];

  return (a + b + c) / 3.0;
}

}  // namespace msd_sim
