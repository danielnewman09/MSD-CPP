#include "RigidBodyProperties.hpp"
#include "ConvexHull.hpp"
#include <stdexcept>
#include <cmath>

namespace msd_sim {

RigidBodyProperties RigidBodyProperties::fromConvexHull(const ConvexHull& hull,
                                                        float mass)
{
  if (!hull.isValid()) {
    throw std::invalid_argument("Cannot create RigidBodyProperties from invalid convex hull");
  }

  if (mass <= 0.0f) {
    throw std::invalid_argument("Mass must be positive");
  }

  // Get hull properties
  float volume = hull.volume();
  if (volume <= 0.0f) {
    throw std::invalid_argument("Convex hull has zero or negative volume");
  }

  Coordinate centerOfMass = hull.centroid();

  // Compute inertia tensor using tetrahedral decomposition
  // Reference: "Explicit Exact Formulas for the 3-D Tetrahedron Inertia Tensor"
  // by F. Tonon (2004)

  const auto& vertices = hull.getVertices();
  const auto& facets = hull.getFacets();

  if (facets.empty()) {
    throw std::invalid_argument("Convex hull has no facets");
  }

  // Accumulate volume-weighted inertia components
  Eigen::Matrix3f inertiaTensor = Eigen::Matrix3f::Zero();
  float totalVolume = 0.0f;
  Coordinate origin(0.0f, 0.0f, 0.0f);

  // For each triangular facet, form a tetrahedron with the origin
  // and compute its contribution to the inertia tensor
  for (const auto& facet : facets) {
    // Get the three vertices of the triangle
    const Coordinate& v0 = vertices[facet.vertexIndices[0]];
    const Coordinate& v1 = vertices[facet.vertexIndices[1]];
    const Coordinate& v2 = vertices[facet.vertexIndices[2]];

    // Translate vertices to center of mass frame
    Coordinate p0 = v0 - centerOfMass;
    Coordinate p1 = v1 - centerOfMass;
    Coordinate p2 = v2 - centerOfMass;

    // Volume of tetrahedron formed by origin and triangle
    float tetVolume = std::abs(p0.dot(p1.cross(p2))) / 6.0f;
    totalVolume += tetVolume;

    // Compute inertia tensor contribution for this tetrahedron
    // Using the formula for inertia tensor of a tetrahedron with one vertex at origin

    // For a tetrahedron with vertices at origin, p0, p1, p2:
    // The inertia tensor components are computed using integral formulas

    // Compute the matrix A = [p0 p1 p2] (each vertex as a column)
    Eigen::Matrix3f A;
    A.col(0) = p0;
    A.col(1) = p1;
    A.col(2) = p2;

    // Compute products of coordinates for this tetrahedron
    // We'll use the standard formula: I = density * integral over volume

    // For a tetrahedron, we can compute:
    // Ixx = density * volume / 60 * (y0² + y1² + y2² + y0*y1 + y0*y2 + y1*y2 + similar for z)

    float x0 = p0.x(), y0 = p0.y(), z0 = p0.z();
    float x1 = p1.x(), y1 = p1.y(), z1 = p1.z();
    float x2 = p2.x(), y2 = p2.y(), z2 = p2.z();

    // Diagonal components (using parallel axis theorem applied to tetrahedron)
    float Ixx = (y0*y0 + y1*y1 + y2*y2 + y0*y1 + y0*y2 + y1*y2 +
                 z0*z0 + z1*z1 + z2*z2 + z0*z1 + z0*z2 + z1*z2);

    float Iyy = (x0*x0 + x1*x1 + x2*x2 + x0*x1 + x0*x2 + x1*x2 +
                 z0*z0 + z1*z1 + z2*z2 + z0*z1 + z0*z2 + z1*z2);

    float Izz = (x0*x0 + x1*x1 + x2*x2 + x0*x1 + x0*x2 + x1*x2 +
                 y0*y0 + y1*y1 + y2*y2 + y0*y1 + y0*y2 + y1*y2);

    // Off-diagonal components (products of inertia)
    float Ixy = -(2.0f*x0*y0 + 2.0f*x1*y1 + 2.0f*x2*y2 +
                  x0*y1 + x1*y0 + x0*y2 + x2*y0 + x1*y2 + x2*y1);

    float Ixz = -(2.0f*x0*z0 + 2.0f*x1*z1 + 2.0f*x2*z2 +
                  x0*z1 + x1*z0 + x0*z2 + x2*z0 + x1*z2 + x2*z1);

    float Iyz = -(2.0f*y0*z0 + 2.0f*y1*z1 + 2.0f*y2*z2 +
                  y0*z1 + y1*z0 + y0*z2 + y2*z0 + y1*z2 + y2*z1);

    // Scale by tetrahedron volume and accumulate
    // The factor 1/60 comes from the integration over the tetrahedron
    float scale = tetVolume / 60.0f;

    inertiaTensor(0, 0) += Ixx * scale;
    inertiaTensor(1, 1) += Iyy * scale;
    inertiaTensor(2, 2) += Izz * scale;
    inertiaTensor(0, 1) += Ixy * scale;
    inertiaTensor(1, 0) += Ixy * scale;
    inertiaTensor(0, 2) += Ixz * scale;
    inertiaTensor(2, 0) += Ixz * scale;
    inertiaTensor(1, 2) += Iyz * scale;
    inertiaTensor(2, 1) += Iyz * scale;
  }

  // Scale inertia tensor by density (mass/volume)
  float density = mass / volume;
  inertiaTensor *= density;

  return RigidBodyProperties(mass, inertiaTensor, centerOfMass);
}

}  // namespace msd_sim
