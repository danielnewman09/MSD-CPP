#include "msd-sim/src/Physics/RigidBody/InertialCalculations.hpp"
#include <cmath>
#include <stdexcept>
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace msd_sim
{
namespace InertialCalculations
{

Eigen::Matrix3d computeInertiaTensorAboutOrigin(const ConvexHull& hull,
                                                 double mass)
{
  if (mass <= 0.0)
  {
    throw std::invalid_argument("Mass must be positive");
  }

  const auto& vertices = hull.getVertices();
  const auto& facets = hull.getFacets();

  if (vertices.empty() || facets.empty())
  {
    return Eigen::Matrix3d::Zero();
  }

  // Compute density from mass and volume
  double vol = hull.getVolume();
  if (vol <= 0.0)
  {
    throw std::runtime_error(
      "Cannot compute inertia tensor for degenerate hull");
  }
  double density = mass / vol;

  // Accumulate inertia tensor contributions from all tetrahedra
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();

  for (const auto& facet : facets)
  {
    const Coordinate& v0 = vertices[facet.vertexIndices[0]];
    const Coordinate& v1 = vertices[facet.vertexIndices[1]];
    const Coordinate& v2 = vertices[facet.vertexIndices[2]];

    // Volume of tetrahedron (with origin as 4th vertex)
    double tetVol = v0.dot(v1.cross(v2)) / 6.0;

    // For a tetrahedron with vertices at origin and v0, v1, v2,
    // we compute the canonical inertia tensor integral

    // Build covariance-like matrix C where C_ij = sum over vertices of v_i *
    // v_j
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();

    // Contribution from each vertex (including origin at 0,0,0)
    // Using the formula for tetrahedron inertia tensor
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        // Sum over all pairs of vertices (includes diagonal terms)
        // For tetrahedron with vertices {0, v0, v1, v2}
        C(i, j) = v0[i] * v0[j] + v1[i] * v1[j] + v2[i] * v2[j] +
                  v0[i] * v1[j] + v0[i] * v2[j] + v1[i] * v0[j] +
                  v1[i] * v2[j] + v2[i] * v0[j] + v2[i] * v1[j];
      }
    }

    // Inertia tensor formula for tetrahedron:
    // I = (density * tetVol / 20) * (trace(C) * I3 - C)
    double scaleFactor = density * std::abs(tetVol) / 20.0;
    double traceC = C.trace();

    Eigen::Matrix3d tetInertia =
      scaleFactor * (traceC * Eigen::Matrix3d::Identity() - C);

    inertia += tetInertia;
  }

  return inertia;
}

Eigen::Matrix3d computeInertiaTensorAboutCentroid(const ConvexHull& hull,
                                                  double mass)
{
  // First compute inertia about origin
  Eigen::Matrix3d I_origin = computeInertiaTensorAboutOrigin(hull, mass);

  // Get the centroid from the hull
  Coordinate com = hull.getCentroid();

  // Apply parallel axis theorem (Steiner's theorem) to shift to centroid
  // I_com = I_origin - m * (r·r * I - r ⊗ r)
  // where r is the position of the centroid
  double r_dot_r = com.dot(com);
  Eigen::Matrix3d r_outer_r = com * com.transpose();

  Eigen::Matrix3d I_centroid =
    I_origin - mass * (r_dot_r * Eigen::Matrix3d::Identity() - r_outer_r);

  return I_centroid;
}

}  // namespace InertialCalculations
}  // namespace msd_sim
