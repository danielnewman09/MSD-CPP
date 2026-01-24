// Prototype P1: EPA Convergence Validation Implementation

#include "epa.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

EPA::EPA(double epsilon) : epsilon_{epsilon} {}

ConvergenceMetrics EPA::computeContactInfoWithMetrics(
    const std::vector<Coordinate>& simplex,
    int maxIterations)
{
  if (simplex.size() != 4) {
    throw std::invalid_argument("EPA requires 4-vertex simplex");
  }

  // Initialize polytope with simplex vertices
  vertices_ = simplex;
  faces_.clear();

  // Create initial 4 faces of tetrahedron
  addFace(0, 1, 2);
  addFace(0, 3, 1);
  addFace(0, 2, 3);
  addFace(1, 3, 2);

  // Expand polytope
  ConvergenceMetrics metrics;
  bool converged = expandPolytope(maxIterations, metrics);
  metrics.converged = converged;

  if (!converged) {
    metrics.penetrationDepth = std::numeric_limits<double>::quiet_NaN();
  }

  return metrics;
}

CollisionResult EPA::computeContactInfo(
    const std::vector<Coordinate>& simplex,
    int maxIterations)
{
  ConvergenceMetrics metrics = computeContactInfoWithMetrics(simplex, maxIterations);

  if (!metrics.converged) {
    throw std::runtime_error("EPA failed to converge within max iterations");
  }

  size_t closestIdx = findClosestFace();
  const EPAFace& closestFace = faces_[closestIdx];

  return CollisionResult{
      closestFace.normal,
      metrics.penetrationDepth,
      computeContactPoint(closestFace)};
}

bool EPA::expandPolytope(int maxIterations, ConvergenceMetrics& metrics) {
  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    metrics.iterations = iteration + 1;

    // Find closest face to origin
    size_t closestIdx = findClosestFace();

    // Check if we have any faces left
    if (faces_.empty()) {
      // Topology corrupted
      return false;
    }

    const EPAFace& closestFace = faces_[closestIdx];

    // Query support point in direction of closest face's normal
    Coordinate newPoint = supportMinkowski(closestFace.normal);

    // Check convergence: if the new support point is not significantly further
    // than the closest face, we've found the boundary
    double newDistance = newPoint.dot(closestFace.normal);
    metrics.finalDistance = closestFace.distance;
    metrics.penetrationDepth = closestFace.distance;

    // Convergence: support point doesn't extend beyond current boundary
    if (newDistance - closestFace.distance < epsilon_) {
      // Converged - closest point found
      return true;
    }

    // Expand polytope
    vertices_.push_back(newPoint);
    std::vector<EPAEdge> horizon = buildHorizonEdges(newPoint);

    // Check for topology failure
    if (horizon.empty()) {
      // No horizon edges - something went wrong
      return false;
    }

    // Create new faces connecting new point to horizon
    for (const auto& edge : horizon) {
      addFace(edge.v0, edge.v1, vertices_.size() - 1);
    }
  }

  // Max iterations reached without convergence
  return false;
}

size_t EPA::findClosestFace() const {
  double minDistance = std::numeric_limits<double>::infinity();
  size_t closestIndex = 0;

  for (size_t i = 0; i < faces_.size(); ++i) {
    if (faces_[i].distance < minDistance) {
      minDistance = faces_[i].distance;
      closestIndex = i;
    }
  }

  return closestIndex;
}

bool EPA::isVisible(const EPAFace& face, const Coordinate& point) const {
  Coordinate toPoint = point - vertices_[face.vertexIndices[0]];
  return face.normal.dot(toPoint) > epsilon_;
}

std::vector<EPAEdge> EPA::buildHorizonEdges(const Coordinate& newVertex) {
  std::vector<EPAEdge> horizon;
  std::vector<size_t> visibleFaceIndices;

  // Identify visible faces
  for (size_t i = 0; i < faces_.size(); ++i) {
    if (isVisible(faces_[i], newVertex)) {
      visibleFaceIndices.push_back(i);
    }
  }

  // Extract edges from visible faces
  std::vector<EPAEdge> edgeCandidates;
  for (size_t idx : visibleFaceIndices) {
    const auto& f = faces_[idx];
    edgeCandidates.emplace_back(f.vertexIndices[0], f.vertexIndices[1]);
    edgeCandidates.emplace_back(f.vertexIndices[1], f.vertexIndices[2]);
    edgeCandidates.emplace_back(f.vertexIndices[2], f.vertexIndices[0]);
  }

  // Horizon edges appear exactly once
  for (const auto& edge : edgeCandidates) {
    int count = std::count(edgeCandidates.begin(), edgeCandidates.end(), edge);
    if (count == 1) {
      horizon.push_back(edge);
    }
  }

  // Remove visible faces
  std::vector<EPAFace> remainingFaces;
  for (size_t i = 0; i < faces_.size(); ++i) {
    if (std::find(visibleFaceIndices.begin(), visibleFaceIndices.end(), i) ==
        visibleFaceIndices.end()) {
      remainingFaces.push_back(faces_[i]);
    }
  }
  faces_ = remainingFaces;

  return horizon;
}

void EPA::addFace(size_t v0, size_t v1, size_t v2) {
  Coordinate a = vertices_[v0];
  Coordinate b = vertices_[v1];
  Coordinate c = vertices_[v2];

  // Compute normal via cross product (right-hand rule)
  Coordinate ab = b - a;
  Coordinate ac = c - a;
  Coordinate normal = ab.cross(ac);

  // Check for degenerate face
  double normalLength = normal.norm();
  if (normalLength < epsilon_) {
    // Degenerate face - skip
    return;
  }

  normal /= normalLength;  // Normalize

  // EPA requires normals to point away from the origin (outward from polytope)
  // Check if normal points toward or away from origin using a point on the face
  if (normal.dot(a) < 0.0) {
    // Normal points toward origin - flip it
    normal = -normal;
    std::swap(v1, v2);  // Maintain winding order consistency
  }

  // Distance from origin to face plane (signed distance along normal)
  // Since normal points outward, distance should be positive
  double distance = normal.dot(a);

  // For polytopes containing the origin, distance should be positive
  // If negative, this face is malformed
  if (distance < 0.0) {
    distance = -distance;
    normal = -normal;
  }

  faces_.emplace_back(v0, v1, v2, normal, distance);
}

Coordinate EPA::computeContactPoint(const EPAFace& face) const {
  // Barycentric centroid
  Coordinate a = vertices_[face.vertexIndices[0]];
  Coordinate b = vertices_[face.vertexIndices[1]];
  Coordinate c = vertices_[face.vertexIndices[2]];

  return (a + b + c) / 3.0;
}

Coordinate EPA::supportMinkowski(const Coordinate& dir) const {
  // For prototype: Simulate Minkowski difference support
  // The Minkowski difference is A - B, where support(A-B, dir) = support(A, dir) - support(B, -dir)

  // For a simplex that contains the origin, we need to find a point on the
  // Minkowski boundary in the direction 'dir'. We simulate this by:
  // 1. Finding the furthest existing vertex in the direction
  // 2. Extending slightly beyond it (simulating actual hull support)

  double maxDot = -std::numeric_limits<double>::infinity();
  Coordinate furthest = vertices_[0];

  for (const auto& vertex : vertices_) {
    double dotProduct = vertex.dot(dir);
    if (dotProduct > maxDot) {
      maxDot = dotProduct;
      furthest = vertex;
    }
  }

  // Extend in the search direction by a small amount proportional to the distance
  // This simulates finding a support point on the actual convex hull boundary
  double extensionFactor = 0.05;
  Coordinate dirNormalized = dir.normalized();
  return furthest + dirNormalized * extensionFactor * furthest.norm();
}
