// Ticket: 0055c_friction_direction_fix
// Design: docs/designs/0055c_friction_direction_fix/design.md

#include "VertexFaceManifoldGenerator.hpp"

#include <algorithm>
#include <cmath>

namespace msd_sim
{

VertexFaceManifoldGenerator::VertexFaceManifoldGenerator(Config config)
  : config_{std::move(config)}
{
}

size_t VertexFaceManifoldGenerator::generate(
  const std::vector<Coordinate>& refFaceVertices,
  const Coordinate& incidentVertex,
  const Vector3D& contactNormal,
  double epaDepth,
  std::array<ContactPoint, 4>& contacts) const
{
  // Validate reference face geometry (must have at least 3 vertices)
  if (refFaceVertices.size() < 3)
  {
    return 0;  // Degenerate face, cannot generate manifold
  }

  // Project reference face vertices onto contact plane
  // Contact plane: contains incidentVertex, perpendicular to contactNormal
  std::vector<Coordinate> projectedVerts =
    projectFaceOntoPlane(refFaceVertices, incidentVertex, contactNormal);

  // Validate projection produced valid geometry
  if (projectedVerts.size() < 3)
  {
    return 0;  // Projection failed, fall back to single-point
  }

  // Build contact pairs using projected vertices
  // Design decision (Option A): All contacts use uniform EPA depth
  size_t contactCount = std::min(projectedVerts.size(), config_.maxContacts);

  for (size_t i = 0; i < contactCount; ++i)
  {
    contacts[i] = ContactPoint{projectedVerts[i], incidentVertex, epaDepth};
  }

  return contactCount;
}

std::vector<Coordinate> VertexFaceManifoldGenerator::projectFaceOntoPlane(
  const std::vector<Coordinate>& faceVertices,
  const Coordinate& planePoint,
  const Vector3D& planeNormal) const
{
  std::vector<Coordinate> projected;
  projected.reserve(faceVertices.size());

  for (const auto& vertex : faceVertices)
  {
    // Project vertex onto contact plane
    // Formula: projectedPoint = vertex - ((vertex - planePoint) · normal) * normal
    Vector3D toVertex{vertex.x() - planePoint.x(),
                      vertex.y() - planePoint.y(),
                      vertex.z() - planePoint.z()};

    double distanceAlongNormal = toVertex.dot(planeNormal);

    Coordinate projectedPoint{vertex.x() - distanceAlongNormal * planeNormal.x(),
                              vertex.y() - distanceAlongNormal * planeNormal.y(),
                              vertex.z() - distanceAlongNormal * planeNormal.z()};

    projected.push_back(projectedPoint);
  }

  return projected;
}

double VertexFaceManifoldGenerator::computePointDepth(
  const Coordinate& refPoint,
  const Coordinate& incidentVertex,
  const Vector3D& contactNormal) const
{
  // Compute per-point penetration depth
  // Depth = (refPoint - incidentVertex) · contactNormal
  Vector3D toRef{refPoint.x() - incidentVertex.x(),
                 refPoint.y() - incidentVertex.y(),
                 refPoint.z() - incidentVertex.z()};

  return toRef.dot(contactNormal);
}

}  // namespace msd_sim
