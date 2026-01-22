// Ticket: 0026_mirtich_inertia_tensor
// Design: docs/designs/0026_mirtich_inertia_tensor/design.md

#include "msd-sim/src/Physics/RigidBody/InertialCalculations.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace msd_sim
{
namespace InertialCalculations
{

namespace
{
// Coordinate indices for projection plane selection
constexpr int X = 0;
constexpr int Y = 1;
constexpr int Z = 2;

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
 * @param[out] A First coordinate index in projection plane
 * @param[out] B Second coordinate index in projection plane
 * @param[out] C Coordinate index perpendicular to projection plane
 * @ticket 0026_mirtich_inertia_tensor
 */
void selectProjectionPlane(const Coordinate& normal, int& A, int& B, int& C)
{
  double nx = std::abs(normal[X]);
  double ny = std::abs(normal[Y]);
  double nz = std::abs(normal[Z]);

  if (nx > ny && nx > nz)
  {
    C = X;
  }
  else if (ny > nz)
  {
    C = Y;
  }
  else
  {
    C = Z;
  }

  A = (C + 1) % 3;
  B = (A + 1) % 3;
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
  const ConvexHull::Facet& facet)
{
  const Coordinate& v0 = vertices[facet.vertexIndices[0]];
  const Coordinate& v1 = vertices[facet.vertexIndices[1]];
  const Coordinate& v2 = vertices[facet.vertexIndices[2]];

  // Compute edge vectors
  Coordinate edge1 = v1 - v0;
  Coordinate edge2 = v2 - v1;

  // Cross product gives normal direction based on vertex winding
  Coordinate computedNormal = edge1.cross(edge2);

  // Check if computed normal aligns with facet normal
  // Positive dot product means same direction, negative means opposite
  double alignment = computedNormal.dot(facet.normal);

  if (alignment >= 0.0)
  {
    // Winding is correct (CCW from outside)
    return facet.vertexIndices;
  }
  else
  {
    // Winding is reversed - swap v1 and v2 to correct
    return {
      facet.vertexIndices[0], facet.vertexIndices[2], facet.vertexIndices[1]};
  }
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
  int A,
  int B)
{
  ProjectionIntegrals proj{};

  // Iterate over edges of the triangular facet
  for (int i = 0; i < 3; ++i)
  {
    int j = (i + 1) % 3;

    const Coordinate& vert_i = vertices[indices[i]];
    const Coordinate& vert_j = vertices[indices[j]];

    double a0 = vert_i[A];
    double b0 = vert_i[B];
    double a1 = vert_j[A];
    double b1 = vert_j[B];

    double da = a1 - a0;
    double db = b1 - b0;

    double a0_2 = a0 * a0;
    double a0_3 = a0_2 * a0;
    double a0_4 = a0_3 * a0;
    double b0_2 = b0 * b0;
    double b0_3 = b0_2 * b0;
    double b0_4 = b0_3 * b0;
    double a1_2 = a1 * a1;
    double a1_3 = a1_2 * a1;
    double b1_2 = b1 * b1;
    double b1_3 = b1_2 * b1;

    double C1 = a1 + a0;
    double Ca = a1 * C1 + a0_2;
    double Caa = a1 * Ca + a0_3;
    double Caaa = a1 * Caa + a0_4;
    double Cb = b1 * (b1 + b0) + b0_2;
    double Cbb = b1 * Cb + b0_3;
    double Cbbb = b1 * Cbb + b0_4;
    double Cab = 3 * a1_2 + 2 * a1 * a0 + a0_2;
    double Kab = a1_2 + 2 * a1 * a0 + 3 * a0_2;
    double Caab = a0 * Cab + 4 * a1_3;
    double Kaab = a1 * Kab + 4 * a0_3;
    double Cabb = 4 * b1_3 + 3 * b1_2 * b0 + 2 * b1 * b0_2 + b0_3;
    double Kabb = b1_3 + 2 * b1_2 * b0 + 3 * b1 * b0_2 + 4 * b0_3;

    proj.P1 += db * C1;
    proj.Pa += db * Ca;
    proj.Paa += db * Caa;
    proj.Paaa += db * Caaa;
    proj.Pb += da * Cb;
    proj.Pbb += da * Cbb;
    proj.Pbbb += da * Cbbb;
    proj.Pab += db * (b1 * Cab + b0 * Kab);
    proj.Paab += db * (b1 * Caab + b0 * Kaab);
    proj.Pabb += da * (a1 * Cabb + a0 * Kabb);
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
 * @param A First projection coordinate index
 * @param B Second projection coordinate index
 * @param C Perpendicular coordinate index
 * @return Face integrals
 * @ticket 0026_mirtich_inertia_tensor
 *
 * Transcribed from volInt.c lines 234-265.
 */
FaceIntegrals computeFaceIntegrals(const std::vector<Coordinate>& vertices,
                                   const std::array<size_t, 3>& indices,
                                   const Coordinate& normal,
                                   const ProjectionIntegrals& proj,
                                   int A,
                                   int B,
                                   int C)
{
  FaceIntegrals face{};

  const Coordinate& n = normal;

  // w = -n · v0 for any vertex v0 on the plane
  const Coordinate& v0 = vertices[indices[0]];
  double w = -(n[X] * v0[X] + n[Y] * v0[Y] + n[Z] * v0[Z]);

  // The projection plane selection ensures n[C] is the largest component,
  // but check for numerical stability
  if (std::abs(n[C]) < 1e-10)
  {
    throw std::runtime_error(
      "Normal component too small - possible degenerate facet");
  }

  double k1 = 1.0 / n[C];
  double k2 = k1 * k1;
  double k3 = k2 * k1;
  double k4 = k3 * k1;

  face.Fa = k1 * proj.Pa;
  face.Fb = k1 * proj.Pb;
  face.Fc = -k2 * (n[A] * proj.Pa + n[B] * proj.Pb + w * proj.P1);

  face.Faa = k1 * proj.Paa;
  face.Fbb = k1 * proj.Pbb;
  face.Fcc = k3 * (n[A] * n[A] * proj.Paa + 2 * n[A] * n[B] * proj.Pab +
                   n[B] * n[B] * proj.Pbb +
                   w * (2 * (n[A] * proj.Pa + n[B] * proj.Pb) + w * proj.P1));

  face.Faaa = k1 * proj.Paaa;
  face.Fbbb = k1 * proj.Pbbb;
  face.Fccc =
    -k4 * (n[A] * n[A] * n[A] * proj.Paaa + 3 * n[A] * n[A] * n[B] * proj.Paab +
           3 * n[A] * n[B] * n[B] * proj.Pabb + n[B] * n[B] * n[B] * proj.Pbbb +
           3 * w *
             (n[A] * n[A] * proj.Paa + 2 * n[A] * n[B] * proj.Pab +
              n[B] * n[B] * proj.Pbb) +
           w * w * (3 * (n[A] * proj.Pa + n[B] * proj.Pb) + w * proj.P1));

  face.Faab = k1 * proj.Paab;
  face.Fbbc = -k2 * (n[A] * proj.Pabb + n[B] * proj.Pbbb + w * proj.Pbb);
  face.Fcca =
    k3 * (n[A] * n[A] * proj.Paaa + 2 * n[A] * n[B] * proj.Paab +
          n[B] * n[B] * proj.Pabb +
          w * (2 * (n[A] * proj.Paa + n[B] * proj.Pab) + w * proj.Pa));

  return face;
}

}  // anonymous namespace

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
  double T0 = 0.0;             // Volume
  std::array<double, 3> T1{};  // First moments
  std::array<double, 3> T2{};  // Second moments
  std::array<double, 3> TP{};  // Products

  // Iterate over all facets and accumulate volume integrals
  for (const auto& facet : facets)
  {
    // Ensure vertex winding is consistent with normal direction
    // This is critical: Qhull provides outward-facing normals, but the vertex
    // order may not match. The Mirtich algorithm requires that the cross
    // product of (v1-v0) x (v2-v1) aligns with the normal.
    std::array<size_t, 3> indices = getWindingCorrectedIndices(vertices, facet);

    const Coordinate& n = facet.normal;

    // Select projection plane based on largest normal component
    int A{}, B{}, C{};
    selectProjectionPlane(n, A, B, C);

    // Compute projection integrals (2D) using winding-corrected indices
    ProjectionIntegrals proj =
      computeProjectionIntegrals(vertices, indices, A, B);

    // Compute face integrals (3D surface)
    FaceIntegrals face =
      computeFaceIntegrals(vertices, indices, n, proj, A, B, C);

    // Accumulate T0 (volume) - transcribed from volInt.c line 291
    T0 += n[X] * ((A == X) ? face.Fa : ((B == X) ? face.Fb : face.Fc));

    // Accumulate T1 (first moments) - transcribed from volInt.c lines 293-295
    T1[A] += n[A] * face.Faa;
    T1[B] += n[B] * face.Fbb;
    T1[C] += n[C] * face.Fcc;

    // Accumulate T2 (second moments) - transcribed from volInt.c lines 296-298
    T2[A] += n[A] * face.Faaa;
    T2[B] += n[B] * face.Fbbb;
    T2[C] += n[C] * face.Fccc;

    // Accumulate TP (products) - transcribed from volInt.c lines 299-301
    TP[A] += n[A] * face.Faab;
    TP[B] += n[B] * face.Fbbc;
    TP[C] += n[C] * face.Fcca;
  }

  // Finalize volume integrals - transcribed from volInt.c lines 304-306
  T1[X] /= 2.0;
  T1[Y] /= 2.0;
  T1[Z] /= 2.0;
  T2[X] /= 3.0;
  T2[Y] /= 3.0;
  T2[Z] /= 3.0;
  TP[X] /= 2.0;
  TP[Y] /= 2.0;
  TP[Z] /= 2.0;

  // Sanity check: volume should be positive
  // (Qhull guarantees outward-facing normals)
  if (T0 <= 0.0)
  {
    throw std::runtime_error("Computed volume is non-positive - possible "
                             "invalid hull or normal orientation issue");
  }

  // Compute density
  double mass = density * T0;

  // Compute inertia tensor about origin - transcribed from volInt.c lines
  // 358-363
  Eigen::Matrix3d I_origin;
  I_origin(X, X) = density * (T2[Y] + T2[Z]);
  I_origin(Y, Y) = density * (T2[Z] + T2[X]);
  I_origin(Z, Z) = density * (T2[X] + T2[Y]);
  I_origin(X, Y) = I_origin(Y, X) = -density * TP[X];
  I_origin(Y, Z) = I_origin(Z, Y) = -density * TP[Y];
  I_origin(Z, X) = I_origin(X, Z) = -density * TP[Z];

  // Compute center of mass
  Coordinate r{T1[X] / T0, T1[Y] / T0, T1[Z] / T0};

  // Apply parallel axis theorem to shift to center of mass - transcribed from
  // volInt.c lines 366-371
  Eigen::Matrix3d I_centroid = I_origin;
  I_centroid(X, X) -= mass * (r[Y] * r[Y] + r[Z] * r[Z]);
  I_centroid(Y, Y) -= mass * (r[Z] * r[Z] + r[X] * r[X]);
  I_centroid(Z, Z) -= mass * (r[X] * r[X] + r[Y] * r[Y]);
  I_centroid(X, Y) = I_centroid(Y, X) += mass * r[X] * r[Y];
  I_centroid(Y, Z) = I_centroid(Z, Y) += mass * r[Y] * r[Z];
  I_centroid(Z, X) = I_centroid(X, Z) += mass * r[Z] * r[X];

  return I_centroid;
}

}  // namespace InertialCalculations
}  // namespace msd_sim
