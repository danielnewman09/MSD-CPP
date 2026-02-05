// Ticket: 0026_mirtich_inertia_tensor
// Design: docs/designs/0026_mirtich_inertia_tensor/design.md

#include "msd-sim/src/Physics/RigidBody/InertialCalculations.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"


namespace msd_sim::inertial_calculations
{

struct ProjectionAxes
{
  Eigen::Index A, B, C;
};

// Helper structures for Mirtich three-layer algorithm

/**
 * @brief Projection integrals computed over 2D polygon projection.
 * @ticket 0026_mirtich_inertia_tensor
 */
struct ProjectionIntegrals
{
  double P1{};
  double Pa{};
  double Pb{};
  double Paa{};
  double Pab{};
  double Pbb{};
  double Paaa{};
  double Paab{};
  double Pabb{};
  double Pbbb{};
};

/**
 * @brief Face integrals computed over 3D surface.
 * @ticket 0026_mirtich_inertia_tensor
 */
struct FaceIntegrals
{
  double Fa{};
  double Fb{};
  double Fc{};
  double Faa{};
  double Fbb{};
  double Fcc{};
  double Faaa{};
  double Fbbb{};
  double Fccc{};
  double Faab{};
  double Fbbc{};
  double Fcca{};
};

/**
 * @brief Select projection plane based on largest normal component.
 * @param normal Facet normal vector
 * @returns The projection axes ordered according to the largest normal
 * @ticket 0026_mirtich_inertia_tensor
 */
ProjectionAxes selectProjectionPlane(const Coordinate& normal)
{
  ProjectionAxes axes{};
  normal.cwiseAbs().maxCoeff(
    &axes.C);  // C = index of largest absolute component

  axes.A = (axes.C + 1) % 3;
  axes.B = (axes.A + 1) % 3;
  return axes;
}

/**
 * @brief Ensure vertex winding is consistent with normal direction.
 *
 * The Mirtich algorithm requires that the cross product of (v1-v0) x (v2-v1)
 * points in the same direction as the facet normal. Qhull provides
 * outward-facing normals but vertex order may not be consistent with that
 * direction.
 *
 * @param vertices Hull vertices
 * @param facet Facet to check
 * @return Vertex indices in corrected winding order
 * @ticket 0026_mirtich_inertia_tensor
 */
std::array<size_t, 3> getWindingCorrectedIndices(
  const std::vector<Coordinate>& vertices,
  const Facet& facet)
{
  const Coordinate& v0 = vertices[facet.vertexIndices[0]];
  const Coordinate& v1 = vertices[facet.vertexIndices[1]];
  const Coordinate& v2 = vertices[facet.vertexIndices[2]];

  // Compute edge vectors
  Coordinate const edge1 = v1 - v0;
  Coordinate const edge2 = v2 - v1;

  // Cross product gives normal direction based on vertex winding
  Coordinate const computedNormal = edge1.cross(edge2);

  // Check if computed normal aligns with facet normal
  // Positive dot product means same direction, negative means opposite
  double const alignment = computedNormal.dot(facet.normal);

  if (alignment >= 0.0)
  {
    // Winding is correct (CCW from outside)
    return facet.vertexIndices;
  }

  // Winding is reversed - swap v1 and v2 to correct
  return {
    facet.vertexIndices[0], facet.vertexIndices[2], facet.vertexIndices[1]};
}

/**
 * @brief Compute projection integrals over 2D polygon projection.
 * @param vertices Hull vertices
 * @param indices Winding-corrected vertex indices
 * @param A First projection coordinate index
 * @param B Second projection coordinate index
 * @return Projection integrals
 * @ticket 0026_mirtich_inertia_tensor
 *
 * Transcribed from volInt.c lines 178-232.
 */
ProjectionIntegrals computeProjectionIntegrals(
  const std::vector<Coordinate>& vertices,
  const std::array<size_t, 3>& indices,
  const ProjectionAxes& axes)
{
  ProjectionIntegrals proj{};

  // Iterate over edges of the triangular facet
  for (size_t i = 0; i < 3; ++i)
  {
    const size_t j = (i + 1) % 3;

    const Coordinate& vertI = vertices[indices[i]];
    const Coordinate& vertJ = vertices[indices[j]];

    const double a0 = vertI[axes.A];
    const double b0 = vertI[axes.B];
    const double a1 = vertJ[axes.A];
    const double b1 = vertJ[axes.B];

    const double da = a1 - a0;
    const double db = b1 - b0;

    const double a02 = a0 * a0;
    const double a03 = a02 * a0;
    const double a04 = a03 * a0;
    const double b02 = b0 * b0;
    const double b03 = b02 * b0;
    const double b04 = b03 * b0;
    const double a12 = a1 * a1;
    const double a13 = a12 * a1;
    const double b12 = b1 * b1;
    const double b13 = b12 * b1;

    const double c1 = a1 + a0;
    const double ca = a1 * c1 + a02;
    const double caa = a1 * ca + a03;
    const double caaa = a1 * caa + a04;
    const double cb = b1 * (b1 + b0) + b02;
    const double cbb = b1 * cb + b03;
    const double cbbb = b1 * cbb + b04;
    const double cab = 3 * a12 + 2 * a1 * a0 + a02;
    const double kab = a12 + 2 * a1 * a0 + 3 * a02;
    const double caab = a0 * cab + 4 * a13;
    const double kaab = a1 * kab + 4 * a03;
    const double cabb = 4 * b13 + 3 * b12 * b0 + 2 * b1 * b02 + b03;
    const double kabb = b13 + 2 * b12 * b0 + 3 * b1 * b02 + 4 * b03;

    proj.P1 += db * c1;
    proj.Pa += db * ca;
    proj.Paa += db * caa;
    proj.Paaa += db * caaa;
    proj.Pb += da * cb;
    proj.Pbb += da * cbb;
    proj.Pbbb += da * cbbb;
    proj.Pab += db * (b1 * cab + b0 * kab);
    proj.Paab += db * (b1 * caab + b0 * kaab);
    proj.Pabb += da * (a1 * cabb + a0 * kabb);
  }

  proj.P1 /= 2.0;
  proj.Pa /= 6.0;
  proj.Paa /= 12.0;
  proj.Paaa /= 20.0;
  proj.Pb /= -6.0;
  proj.Pbb /= -12.0;
  proj.Pbbb /= -20.0;
  proj.Pab /= 24.0;
  proj.Paab /= 60.0;
  proj.Pabb /= -60.0;

  return proj;
}

/**
 * @brief Compute face integrals from projection integrals.
 * @param vertices Hull vertices
 * @param indices Winding-corrected vertex indices
 * @param normal Facet normal
 * @param proj Projection integrals
 * @param axes The projection axes
 * @return Face integrals
 * @ticket 0026_mirtich_inertia_tensor
 *
 * Transcribed from volInt.c lines 234-265.
 */
FaceIntegrals computeFaceIntegrals(const std::vector<Coordinate>& vertices,
                                   const std::array<size_t, 3>& indices,
                                   const Coordinate& normal,
                                   const ProjectionIntegrals& proj,
                                   const ProjectionAxes& axes)
{
  FaceIntegrals face{};

  const Coordinate& n = normal;

  // w = -n · v0 for any vertex v0 on the plane
  const Coordinate& v0 = vertices[indices[0]];
  const double w = -((n[Coordinate::X] * v0[Coordinate::X]) +
                     (n[Coordinate::Y] * v0[Coordinate::Y]) +
                     (n[Coordinate::Z] * v0[Coordinate::Z]));

  // The projection plane selection ensures n[C] is the largest component,
  // but check for numerical stability
  if (std::abs(n[axes.C]) < 1e-10)
  {
    throw std::runtime_error(
      "Normal component too small - possible degenerate facet");
  }

  const double k1 = 1.0 / n[axes.C];
  const double k2 = k1 * k1;
  const double k3 = k2 * k1;
  const double k4 = k3 * k1;

  face.Fa = k1 * proj.Pa;
  face.Fb = k1 * proj.Pb;
  face.Fc = -k2 * (n[axes.A] * proj.Pa + n[axes.B] * proj.Pb + w * proj.P1);

  face.Faa = k1 * proj.Paa;
  face.Fbb = k1 * proj.Pbb;
  face.Fcc =
    k3 *
    (n[axes.A] * n[axes.A] * proj.Paa + 2 * n[axes.A] * n[axes.B] * proj.Pab +
     n[axes.B] * n[axes.B] * proj.Pbb +
     w * (2 * (n[axes.A] * proj.Pa + n[axes.B] * proj.Pb) + w * proj.P1));

  face.Faaa = k1 * proj.Paaa;
  face.Fbbb = k1 * proj.Pbbb;
  face.Fccc =
    -k4 *
    (n[axes.A] * n[axes.A] * n[axes.A] * proj.Paaa +
     3 * n[axes.A] * n[axes.A] * n[axes.B] * proj.Paab +
     3 * n[axes.A] * n[axes.B] * n[axes.B] * proj.Pabb +
     n[axes.B] * n[axes.B] * n[axes.B] * proj.Pbbb +
     3 * w *
       (n[axes.A] * n[axes.A] * proj.Paa +
        2 * n[axes.A] * n[axes.B] * proj.Pab +
        n[axes.B] * n[axes.B] * proj.Pbb) +
     w * w * (3 * (n[axes.A] * proj.Pa + n[axes.B] * proj.Pb) + w * proj.P1));

  face.Faab = k1 * proj.Paab;
  face.Fbbc =
    -k2 * (n[axes.A] * proj.Pabb + n[axes.B] * proj.Pbbb + w * proj.Pbb);
  face.Fcca =
    k3 *
    (n[axes.A] * n[axes.A] * proj.Paaa + 2 * n[axes.A] * n[axes.B] * proj.Paab +
     n[axes.B] * n[axes.B] * proj.Pabb +
     w * (2 * (n[axes.A] * proj.Paa + n[axes.B] * proj.Pab) + w * proj.Pa));

  return face;
}

Eigen::Matrix3d computeInertiaTensorAboutCentroid(const ConvexHull& hull,
                                                  double density)
{
  // Ticket: 0026_mirtich_inertia_tensor
  // Design: docs/designs/0026_mirtich_inertia_tensor/design.md
  //
  // Implementation of Brian Mirtich's algorithm from "Fast and Accurate
  // Computation of Polyhedral Mass Properties" (1996).
  //
  // Uses divergence theorem to convert volume integrals to surface integrals
  // through three layers:
  //   1. Projection integrals (2D) over facet projections
  //   2. Face integrals (3D surface) lifted from projection integrals
  //   3. Volume integrals accumulated across all facets
  //
  // This replaces the previous tetrahedron decomposition which had ~10-15%
  // accuracy error. The Mirtich algorithm is mathematically exact and produces
  // results within machine precision of analytical solutions.

  if (density <= 0.0)
  {
    throw std::invalid_argument("Mass must be positive");
  }

  if (!hull.isValid())
  {
    throw std::runtime_error("Cannot compute inertia tensor for invalid hull");
  }

  const auto& vertices = hull.getVertices();
  const auto& facets = hull.getFacets();

  if (vertices.empty() || facets.empty())
  {
    return Eigen::Matrix3d::Zero();
  }

  // Initialize volume integral accumulators
  // double T0 = 0.0;  // Volume
  double const volume = hull.getVolume();
  Coordinate t1;    // First moments
  Coordinate t2{};  // Second moments
  Coordinate tp{};  // Products

  // Iterate over all facets and accumulate volume integrals
  for (const auto& facet : facets)
  {
    // Ensure vertex winding is consistent with normal direction
    // This is critical: Qhull provides outward-facing normals, but the vertex
    // order may not match. The Mirtich algorithm requires that the cross
    // product of (v1-v0) x (v2-v1) aligns with the normal.
    std::array<size_t, 3> const indices =
      inertial_calculations::getWindingCorrectedIndices(vertices, facet);

    const Coordinate& n = facet.normal;

    // Select projection plane based on largest normal component
    inertial_calculations::ProjectionAxes const axes =
      inertial_calculations::selectProjectionPlane(n);

    // Compute projection integrals (2D) using winding-corrected indices
    inertial_calculations::ProjectionIntegrals const proj =
      inertial_calculations::computeProjectionIntegrals(
        vertices, indices, axes);

    // Compute face integrals (3D surface)
    inertial_calculations::FaceIntegrals const face =
      inertial_calculations::computeFaceIntegrals(
        vertices, indices, n, proj, axes);

    // Accumulate T0 (volume) - transcribed from volInt.c line 291
    // T0 += n[X] * ((A == X) ? face.Fa : ((B == X) ? face.Fb : face.Fc));


    // Accumulate T1 (first moments) - transcribed from volInt.c lines 293-295
    t1[axes.A] += n[axes.A] * face.Faa;
    t1[axes.B] += n[axes.B] * face.Fbb;
    t1[axes.C] += n[axes.C] * face.Fcc;

    // Accumulate T2 (second moments) - transcribed from volInt.c lines 296-298
    t2[axes.A] += n[axes.A] * face.Faaa;
    t2[axes.B] += n[axes.B] * face.Fbbb;
    t2[axes.C] += n[axes.C] * face.Fccc;

    // Accumulate TP (products) - transcribed from volInt.c lines 299-301
    tp[axes.A] += n[axes.A] * face.Faab;
    tp[axes.B] += n[axes.B] * face.Fbbc;
    tp[axes.C] += n[axes.C] * face.Fcca;
  }

  // Finalize volume integrals - transcribed from volInt.c lines 304-306
  t1[Coordinate::X] /= 2.0;
  t1[Coordinate::Y] /= 2.0;
  t1[Coordinate::Z] /= 2.0;
  t2[Coordinate::X] /= 3.0;
  t2[Coordinate::Y] /= 3.0;
  t2[Coordinate::Z] /= 3.0;
  tp[Coordinate::X] /= 2.0;
  tp[Coordinate::Y] /= 2.0;
  tp[Coordinate::Z] /= 2.0;

  // Sanity check: volume should be positive
  // (Qhull guarantees outward-facing normals)
  if (volume <= 0.0)
  {
    throw std::runtime_error("Computed volume is non-positive - possible "
                             "invalid hull or normal orientation issue");
  }

  // Compute density
  double const mass = density * volume;

  // Compute inertia tensor about origin - transcribed from volInt.c lines
  // 358-363
  Eigen::Matrix3d iOrigin;
  iOrigin(Coordinate::X, Coordinate::X) =
    density * (t2[Coordinate::Y] + t2[Coordinate::Z]);
  iOrigin(Coordinate::Y, Coordinate::Y) =
    density * (t2[Coordinate::Z] + t2[Coordinate::X]);
  iOrigin(Coordinate::Z, Coordinate::Z) =
    density * (t2[Coordinate::X] + t2[Coordinate::Y]);
  iOrigin(Coordinate::X, Coordinate::Y) =
    iOrigin(Coordinate::Y, Coordinate::X) = -density * tp[Coordinate::X];
  iOrigin(Coordinate::Y, Coordinate::Z) =
    iOrigin(Coordinate::Z, Coordinate::Y) = -density * tp[Coordinate::Y];
  iOrigin(Coordinate::Z, Coordinate::X) =
    iOrigin(Coordinate::X, Coordinate::Z) = -density * tp[Coordinate::Z];

  // Compute center of mass
  Coordinate r{t1[Coordinate::X] / volume,
               t1[Coordinate::Y] / volume,
               t1[Coordinate::Z] / volume};

  // Apply parallel axis theorem to shift to center of mass - transcribed from
  // volInt.c lines 366-371
  Eigen::Matrix3d iCentroid = iOrigin;
  iCentroid(Coordinate::X, Coordinate::X) -=
    mass *
    (r[Coordinate::Y] * r[Coordinate::Y] + r[Coordinate::Z] * r[Coordinate::Z]);
  iCentroid(Coordinate::Y, Coordinate::Y) -=
    mass *
    (r[Coordinate::Z] * r[Coordinate::Z] + r[Coordinate::X] * r[Coordinate::X]);
  iCentroid(Coordinate::Z, Coordinate::Z) -=
    mass *
    (r[Coordinate::X] * r[Coordinate::X] + r[Coordinate::Y] * r[Coordinate::Y]);
  iCentroid(Coordinate::X, Coordinate::Y) = iCentroid(
    Coordinate::Y, Coordinate::X) += mass * r[Coordinate::X] * r[Coordinate::Y];
  iCentroid(Coordinate::Y, Coordinate::Z) = iCentroid(
    Coordinate::Z, Coordinate::Y) += mass * r[Coordinate::Y] * r[Coordinate::Z];
  iCentroid(Coordinate::Z, Coordinate::X) = iCentroid(
    Coordinate::X, Coordinate::Z) += mass * r[Coordinate::Z] * r[Coordinate::X];

  return iCentroid;
}
}  // namespace msd_sim::inertial_calculations
