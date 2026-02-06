/**
 * @file example.cpp
 * @brief Tutorial implementation of Mirtich's inertia tensor algorithm
 *
 * This file demonstrates the computation of moment of inertia tensors for
 * convex polyhedra using Brian Mirtich's algorithm from "Fast and Accurate
 * Computation of Polyhedral Mass Properties" (1996).
 *
 * Production equivalent:
 * msd/msd-sim/src/Physics/RigidBody/InertialCalculations.cpp The production
 * version:
 * - Uses Eigen for matrix operations
 * - Integrates with ConvexHull and Qhull
 * - Includes vertex winding correction for Qhull compatibility
 * - Has comprehensive error handling
 * This tutorial version prioritizes readability and self-containment.
 *
 * @see README.md for algorithm explanation
 * @see source_code/volInt.c for Mirtich's original reference implementation
 *
 * Ticket: 0026_mirtich_inertia_tensor
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// =============================================================================
// Constants
// =============================================================================

/// Coordinate axis indices for projection plane selection
constexpr int X = 0;
constexpr int Y = 1;
constexpr int Z = 2;

// =============================================================================
// Data Structures
// =============================================================================

/**
 * @brief Simple 3D vector for tutorial purposes
 *
 * Production equivalent: msd_sim::Coordinate (msd_sim::Vector3D wrapper)
 */
struct Vec3
{
  double x, y, z;

  Vec3() : x{0.0}, y{0.0}, z{0.0}
  {
  }

  Vec3(double x_, double y_, double z_) : x{x_}, y{y_}, z{z_}
  {
  }

  /// Access by index (0=x, 1=y, 2=z)
  double operator[](int i) const
  {
    if (i == 0)
      return x;
    if (i == 1)
      return y;
    return z;
  }

  /// Dot product: a · b = ax*bx + ay*by + az*bz
  double dot(const Vec3& other) const
  {
    return x * other.x + y * other.y + z * other.z;
  }

  /// Euclidean length: |v| = sqrt(x² + y² + z²)
  double length() const
  {
    return std::sqrt(x * x + y * y + z * z);
  }

  /// Return unit vector in same direction
  Vec3 normalized() const
  {
    double len = length();
    return Vec3{x / len, y / len, z / len};
  }

  /// Cross product: a × b
  Vec3 cross(const Vec3& other) const
  {
    return Vec3{y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x};
  }
};

/**
 * @brief 3x3 matrix for inertia tensor
 *
 * Production equivalent: Eigen::Matrix3d
 */
struct Mat3
{
  double data[3][3];

  Mat3()
  {
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        data[i][j] = 0.0;
      }
    }
  }

  double& operator()(int i, int j)
  {
    return data[i][j];
  }
  double operator()(int i, int j) const
  {
    return data[i][j];
  }

  void print() const
  {
    for (int i = 0; i < 3; ++i)
    {
      std::cout << "[";
      for (int j = 0; j < 3; ++j)
      {
        std::cout << std::setw(15) << std::setprecision(10) << data[i][j];
        if (j < 2)
          std::cout << ", ";
      }
      std::cout << "]\n";
    }
  }
};

/**
 * @brief Triangular facet with vertex indices and outward normal
 *
 * The Mirtich algorithm requires that vertices are ordered counter-clockwise
 * when viewed from outside the polyhedron (CCW from outside).
 */
struct Facet
{
  std::array<int, 3> vertexIndices;
  Vec3 normal;
};

/**
 * @brief Simple polyhedral mesh: vertices + triangular facets
 *
 * Production equivalent: msd_sim::ConvexHull
 */
struct Mesh
{
  std::vector<Vec3> vertices;
  std::vector<Facet> facets;

  /**
   * @brief Compute facet normals from vertex winding
   *
   * For CCW winding (viewed from outside), the cross product of consecutive
   * edges gives an outward-facing normal:
   *   edge1 = v1 - v0
   *   edge2 = v2 - v1
   *   normal = edge1 × edge2 (then normalized)
   */
  void computeFacetNormals()
  {
    for (auto& facet : facets)
    {
      const Vec3& v0 = vertices[facet.vertexIndices[0]];
      const Vec3& v1 = vertices[facet.vertexIndices[1]];
      const Vec3& v2 = vertices[facet.vertexIndices[2]];

      Vec3 edge1{v1.x - v0.x, v1.y - v0.y, v1.z - v0.z};
      Vec3 edge2{v2.x - v1.x, v2.y - v1.y, v2.z - v1.z};

      facet.normal = edge1.cross(edge2).normalized();
    }
  }
};

// =============================================================================
// Mirtich Algorithm - Layer 1: Projection Integrals
// =============================================================================

/**
 * @brief Projection integrals computed over 2D polygon projection
 *
 * These are line integrals around the polygon edges in a 2D projection plane.
 * The integrals capture area and moments of the projected polygon.
 *
 * Transcribed from volInt.c lines 178-232.
 */
struct ProjectionIntegrals
{
  double P1;                      // ∫1 dA (signed area)
  double Pa, Pb;                  // ∫a dA, ∫b dA (first moments)
  double Paa, Pab, Pbb;           // ∫a² dA, ∫ab dA, ∫b² dA (second moments)
  double Paaa, Paab, Pabb, Pbbb;  // Third moments
};

/**
 * @brief Compute projection integrals for a triangular facet
 *
 * Projects the facet onto a 2D plane (A-B plane, perpendicular to C axis)
 * and computes line integrals around the triangle edges using Green's theorem.
 *
 * @param mesh The polyhedral mesh
 * @param facet The facet to process
 * @param A First coordinate index in projection plane
 * @param B Second coordinate index in projection plane
 * @return Projection integrals
 */
ProjectionIntegrals computeProjectionIntegrals(const Mesh& mesh,
                                               const Facet& facet,
                                               int A,
                                               int B)
{
  ProjectionIntegrals proj{};
  proj.P1 = proj.Pa = proj.Pb = 0.0;
  proj.Paa = proj.Pab = proj.Pbb = 0.0;
  proj.Paaa = proj.Paab = proj.Pabb = proj.Pbbb = 0.0;

  // Iterate over the three edges of the triangle
  for (int i = 0; i < 3; ++i)
  {
    int j = (i + 1) % 3;

    const Vec3& vert_i = mesh.vertices[facet.vertexIndices[i]];
    const Vec3& vert_j = mesh.vertices[facet.vertexIndices[j]];

    // Edge endpoints in projection plane
    double a0 = vert_i[A];
    double b0 = vert_i[B];
    double a1 = vert_j[A];
    double b1 = vert_j[B];

    // Edge deltas
    double da = a1 - a0;
    double db = b1 - b0;

    // Precompute powers for efficiency
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

    // Accumulation coefficients (from Mirtich's derivation)
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

    // Accumulate line integrals
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

  // Apply final scaling factors
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

// =============================================================================
// Mirtich Algorithm - Layer 2: Face Integrals
// =============================================================================

/**
 * @brief Face integrals computed over 3D surface
 *
 * These are surface integrals over the facet, lifted from the 2D projection
 * integrals using the facet plane equation: n·x + w = 0
 *
 * Transcribed from volInt.c lines 234-265.
 */
struct FaceIntegrals
{
  double Fa, Fb, Fc;        // First moments: ∫a, ∫b, ∫c over face
  double Faa, Fbb, Fcc;     // Second moments
  double Faaa, Fbbb, Fccc;  // Third moments
  double Faab, Fbbc, Fcca;  // Mixed third moments
};

/**
 * @brief Compute face integrals from projection integrals
 *
 * Lifts the 2D projection integrals to 3D surface integrals using the
 * relationship between the projection plane and the facet plane.
 *
 * @param mesh The polyhedral mesh
 * @param facet The facet to process
 * @param proj Projection integrals from Layer 1
 * @param A First coordinate index in projection plane
 * @param B Second coordinate index in projection plane
 * @param C Coordinate index perpendicular to projection plane
 * @return Face integrals
 */
FaceIntegrals computeFaceIntegrals(const Mesh& mesh,
                                   const Facet& facet,
                                   const ProjectionIntegrals& proj,
                                   int A,
                                   int B,
                                   int C)
{
  FaceIntegrals face{};

  const Vec3& n = facet.normal;

  // w = -n · v0 for any vertex v0 on the plane (plane equation: n·x + w = 0)
  const Vec3& v0 = mesh.vertices[facet.vertexIndices[0]];
  double w = -(n[X] * v0[X] + n[Y] * v0[Y] + n[Z] * v0[Z]);

  // k values for lifting from projection plane to 3D
  // These account for the plane's tilt relative to the projection plane
  double k1 = 1.0 / n[C];
  double k2 = k1 * k1;
  double k3 = k2 * k1;
  double k4 = k3 * k1;

  // First moments
  face.Fa = k1 * proj.Pa;
  face.Fb = k1 * proj.Pb;
  face.Fc = -k2 * (n[A] * proj.Pa + n[B] * proj.Pb + w * proj.P1);

  // Second moments
  face.Faa = k1 * proj.Paa;
  face.Fbb = k1 * proj.Pbb;
  face.Fcc = k3 * (n[A] * n[A] * proj.Paa + 2 * n[A] * n[B] * proj.Pab +
                   n[B] * n[B] * proj.Pbb +
                   w * (2 * (n[A] * proj.Pa + n[B] * proj.Pb) + w * proj.P1));

  // Third moments
  face.Faaa = k1 * proj.Paaa;
  face.Fbbb = k1 * proj.Pbbb;
  face.Fccc =
    -k4 * (n[A] * n[A] * n[A] * proj.Paaa + 3 * n[A] * n[A] * n[B] * proj.Paab +
           3 * n[A] * n[B] * n[B] * proj.Pabb + n[B] * n[B] * n[B] * proj.Pbbb +
           3 * w *
             (n[A] * n[A] * proj.Paa + 2 * n[A] * n[B] * proj.Pab +
              n[B] * n[B] * proj.Pbb) +
           w * w * (3 * (n[A] * proj.Pa + n[B] * proj.Pb) + w * proj.P1));

  // Mixed third moments
  face.Faab = k1 * proj.Paab;
  face.Fbbc = -k2 * (n[A] * proj.Pabb + n[B] * proj.Pbbb + w * proj.Pbb);
  face.Fcca =
    k3 * (n[A] * n[A] * proj.Paaa + 2 * n[A] * n[B] * proj.Paab +
          n[B] * n[B] * proj.Pabb +
          w * (2 * (n[A] * proj.Paa + n[B] * proj.Pab) + w * proj.Pa));

  return face;
}

// =============================================================================
// Mirtich Algorithm - Layer 3: Volume Integrals
// =============================================================================

/**
 * @brief Volume integrals accumulated across all faces
 *
 * These are the final volume integrals from which mass properties are derived.
 */
struct VolumeIntegrals
{
  double T0;                 // Volume: ∫∫∫ 1 dV
  std::array<double, 3> T1;  // First moments: ∫∫∫ x, y, z dV
  std::array<double, 3> T2;  // Second moments: ∫∫∫ x², y², z² dV
  std::array<double, 3> TP;  // Products: ∫∫∫ xy, yz, zx dV
};

/**
 * @brief Select projection plane based on largest normal component
 *
 * To avoid numerical instability, we project onto the plane where the
 * facet normal has its largest component. This ensures we never divide
 * by a near-zero value when lifting back to 3D.
 *
 * @param normal Facet normal vector
 * @param[out] A First coordinate index in projection plane
 * @param[out] B Second coordinate index in projection plane
 * @param[out] C Coordinate index perpendicular to projection plane (largest
 * |n|)
 */
void selectProjectionPlane(const Vec3& normal, int& A, int& B, int& C)
{
  double nx = std::abs(normal[X]);
  double ny = std::abs(normal[Y]);
  double nz = std::abs(normal[Z]);

  if (nx > ny && nx > nz)
  {
    C = X;  // Project onto YZ plane
  }
  else if (ny > nz)
  {
    C = Y;  // Project onto XZ plane
  }
  else
  {
    C = Z;  // Project onto XY plane
  }

  // A and B are the other two axes (cyclic order)
  A = (C + 1) % 3;
  B = (A + 1) % 3;
}

/**
 * @brief Compute volume integrals for entire polyhedron
 *
 * Iterates over all facets, computing projection and face integrals,
 * and accumulating into volume integrals.
 *
 * Transcribed from volInt.c lines 267-307.
 *
 * @param mesh The polyhedral mesh
 * @return Volume integrals (T0, T1, T2, TP)
 */
VolumeIntegrals computeVolumeIntegrals(const Mesh& mesh)
{
  VolumeIntegrals vol{};
  vol.T0 = 0.0;
  vol.T1 = {0.0, 0.0, 0.0};
  vol.T2 = {0.0, 0.0, 0.0};
  vol.TP = {0.0, 0.0, 0.0};

  for (const auto& facet : mesh.facets)
  {
    // Layer 1: Select projection plane
    int A, B, C;
    selectProjectionPlane(facet.normal, A, B, C);

    // Layer 2: Compute projection integrals (2D)
    ProjectionIntegrals proj = computeProjectionIntegrals(mesh, facet, A, B);

    // Layer 3: Lift to face integrals (3D surface)
    FaceIntegrals face = computeFaceIntegrals(mesh, facet, proj, A, B, C);

    const Vec3& n = facet.normal;

    // Accumulate T0 (volume) using divergence theorem
    // T0 = ∫∫∫ 1 dV = ∫∫ x·nx dA (choosing x component)
    vol.T0 += n[X] * ((A == X) ? face.Fa : ((B == X) ? face.Fb : face.Fc));

    // Accumulate T1 (first moments)
    vol.T1[A] += n[A] * face.Faa;
    vol.T1[B] += n[B] * face.Fbb;
    vol.T1[C] += n[C] * face.Fcc;

    // Accumulate T2 (second moments)
    vol.T2[A] += n[A] * face.Faaa;
    vol.T2[B] += n[B] * face.Fbbb;
    vol.T2[C] += n[C] * face.Fccc;

    // Accumulate TP (products)
    vol.TP[A] += n[A] * face.Faab;
    vol.TP[B] += n[B] * face.Fbbc;
    vol.TP[C] += n[C] * face.Fcca;
  }

  // Apply final scaling factors
  vol.T1[X] /= 2.0;
  vol.T1[Y] /= 2.0;
  vol.T1[Z] /= 2.0;
  vol.T2[X] /= 3.0;
  vol.T2[Y] /= 3.0;
  vol.T2[Z] /= 3.0;
  vol.TP[X] /= 2.0;
  vol.TP[Y] /= 2.0;
  vol.TP[Z] /= 2.0;

  return vol;
}

// =============================================================================
// Final Computation: Inertia Tensor About Centroid
// =============================================================================

/**
 * @brief Compute inertia tensor about center of mass
 *
 * This is the main entry point that:
 * 1. Computes volume integrals
 * 2. Derives inertia about origin from second moments and products
 * 3. Applies parallel axis theorem to shift to center of mass
 *
 * Transcribed from volInt.c lines 358-371.
 *
 * @param mesh The polyhedral mesh
 * @param mass Total mass of the object
 * @return 3x3 inertia tensor about center of mass
 */
Mat3 computeInertiaTensorAboutCentroid(const Mesh& mesh, double mass)
{
  VolumeIntegrals vol = computeVolumeIntegrals(mesh);

  // Compute density from mass and volume
  double density = mass / vol.T0;

  std::cout << "  Volume: " << vol.T0 << "\n";

  // Compute inertia about origin (from Mirtich's derivation)
  // Ixx = ρ ∫∫∫(y² + z²) dV = ρ(T2[Y] + T2[Z])
  Mat3 I_origin;
  I_origin(X, X) = density * (vol.T2[Y] + vol.T2[Z]);
  I_origin(Y, Y) = density * (vol.T2[Z] + vol.T2[X]);
  I_origin(Z, Z) = density * (vol.T2[X] + vol.T2[Y]);
  I_origin(X, Y) = I_origin(Y, X) = -density * vol.TP[X];
  I_origin(Y, Z) = I_origin(Z, Y) = -density * vol.TP[Y];
  I_origin(Z, X) = I_origin(X, Z) = -density * vol.TP[Z];

  // Compute center of mass
  Vec3 r{vol.T1[X] / vol.T0, vol.T1[Y] / vol.T0, vol.T1[Z] / vol.T0};
  std::cout << "  Center of mass: (" << r.x << ", " << r.y << ", " << r.z
            << ")\n";

  // Apply parallel axis theorem to shift to center of mass
  // I_cm = I_origin - m * (r·r * I - r ⊗ r)
  Mat3 I_centroid = I_origin;
  I_centroid(X, X) -= mass * (r.y * r.y + r.z * r.z);
  I_centroid(Y, Y) -= mass * (r.z * r.z + r.x * r.x);
  I_centroid(Z, Z) -= mass * (r.x * r.x + r.y * r.y);
  I_centroid(X, Y) = I_centroid(Y, X) += mass * r.x * r.y;
  I_centroid(Y, Z) = I_centroid(Z, Y) += mass * r.y * r.z;
  I_centroid(Z, X) = I_centroid(X, Z) += mass * r.z * r.x;

  return I_centroid;
}

// =============================================================================
// Test Geometry Factories
// =============================================================================

/**
 * @brief Create unit cube mesh (1×1×1 centered at origin)
 *
 * Vertices ordered with CCW winding when viewed from outside.
 * 12 triangular facets (2 per face).
 */
Mesh createUnitCube()
{
  Mesh mesh;

  // 8 vertices of unit cube centered at origin
  mesh.vertices = {Vec3{-0.5, -0.5, -0.5},
                   Vec3{0.5, -0.5, -0.5},
                   Vec3{0.5, 0.5, -0.5},
                   Vec3{-0.5, 0.5, -0.5},
                   Vec3{-0.5, -0.5, 0.5},
                   Vec3{0.5, -0.5, 0.5},
                   Vec3{0.5, 0.5, 0.5},
                   Vec3{-0.5, 0.5, 0.5}};

  // 12 triangular facets with CCW winding (viewed from outside)
  mesh.facets = {// Bottom (-Z face)
                 {{0, 2, 1}, Vec3{}},
                 {{0, 3, 2}, Vec3{}},
                 // Top (+Z face)
                 {{4, 5, 6}, Vec3{}},
                 {{4, 6, 7}, Vec3{}},
                 // Front (-Y face)
                 {{0, 1, 5}, Vec3{}},
                 {{0, 5, 4}, Vec3{}},
                 // Back (+Y face)
                 {{2, 3, 7}, Vec3{}},
                 {{2, 7, 6}, Vec3{}},
                 // Left (-X face)
                 {{0, 4, 7}, Vec3{}},
                 {{0, 7, 3}, Vec3{}},
                 // Right (+X face)
                 {{1, 2, 6}, Vec3{}},
                 {{1, 6, 5}, Vec3{}}};

  mesh.computeFacetNormals();
  return mesh;
}

/**
 * @brief Create rectangular box mesh (a×b×c centered at origin)
 */
Mesh createRectangularBox(double a, double b, double c)
{
  Mesh mesh;

  double hx = a / 2.0;
  double hy = b / 2.0;
  double hz = c / 2.0;

  mesh.vertices = {Vec3{-hx, -hy, -hz},
                   Vec3{hx, -hy, -hz},
                   Vec3{hx, hy, -hz},
                   Vec3{-hx, hy, -hz},
                   Vec3{-hx, -hy, hz},
                   Vec3{hx, -hy, hz},
                   Vec3{hx, hy, hz},
                   Vec3{-hx, hy, hz}};

  mesh.facets = {{{0, 2, 1}, Vec3{}},
                 {{0, 3, 2}, Vec3{}},
                 {{4, 5, 6}, Vec3{}},
                 {{4, 6, 7}, Vec3{}},
                 {{0, 1, 5}, Vec3{}},
                 {{0, 5, 4}, Vec3{}},
                 {{2, 3, 7}, Vec3{}},
                 {{2, 7, 6}, Vec3{}},
                 {{0, 4, 7}, Vec3{}},
                 {{0, 7, 3}, Vec3{}},
                 {{1, 2, 6}, Vec3{}},
                 {{1, 6, 5}, Vec3{}}};

  mesh.computeFacetNormals();
  return mesh;
}

/**
 * @brief Create regular tetrahedron mesh (centered at origin)
 *
 * Vertices placed at symmetric positions equidistant from origin.
 */
Mesh createRegularTetrahedron(double edgeLength)
{
  Mesh mesh;

  // Regular tetrahedron vertices at unit distance from origin
  // These form an equilateral tetrahedron when scaled appropriately
  double scale = edgeLength / (2.0 * std::sqrt(2.0));

  mesh.vertices = {Vec3{scale, scale, scale},
                   Vec3{scale, -scale, -scale},
                   Vec3{-scale, scale, -scale},
                   Vec3{-scale, -scale, scale}};

  // Facets with CCW winding (viewed from outside)
  mesh.facets = {
    {{0, 1, 2}, Vec3{}},  // Face opposite vertex 3
    {{0, 3, 1}, Vec3{}},  // Face opposite vertex 2
    {{0, 2, 3}, Vec3{}},  // Face opposite vertex 1
    {{1, 3, 2}, Vec3{}}   // Face opposite vertex 0
  };

  mesh.computeFacetNormals();
  return mesh;
}

// =============================================================================
// Test Suite
// =============================================================================

void testUnitCube()
{
  std::cout << "\n=== Test: Unit Cube (1x1x1) ===\n";

  Mesh cube = createUnitCube();
  double mass = 1.0;

  Mat3 I = computeInertiaTensorAboutCentroid(cube, mass);

  // Analytical solution: Ixx = Iyy = Izz = m/6 for unit cube
  double expected_diagonal = mass / 6.0;

  std::cout << "Computed inertia tensor:\n";
  I.print();

  std::cout << "\nExpected diagonal: " << expected_diagonal << "\n";
  std::cout << "Expected off-diagonal: 0.0\n";

  double error_xx = std::abs(I(X, X) - expected_diagonal);
  double error_yy = std::abs(I(Y, Y) - expected_diagonal);
  double error_zz = std::abs(I(Z, Z) - expected_diagonal);

  std::cout << "\nErrors: Ixx=" << error_xx << ", Iyy=" << error_yy
            << ", Izz=" << error_zz << "\n";

  bool passed = (error_xx < 1e-10 && error_yy < 1e-10 && error_zz < 1e-10);
  std::cout << "Result: " << (passed ? "PASS" : "FAIL") << "\n";
}

void testRectangularBox()
{
  std::cout << "\n=== Test: Rectangular Box (2x3x4) ===\n";

  double a = 2.0, b = 3.0, c = 4.0;
  Mesh box = createRectangularBox(a, b, c);
  double mass = 1.0;

  Mat3 I = computeInertiaTensorAboutCentroid(box, mass);

  // Analytical solution for rectangular box about centroid
  double expected_xx = mass * (b * b + c * c) / 12.0;
  double expected_yy = mass * (a * a + c * c) / 12.0;
  double expected_zz = mass * (a * a + b * b) / 12.0;

  std::cout << "Computed inertia tensor:\n";
  I.print();

  std::cout << "\nExpected: Ixx=" << expected_xx << ", Iyy=" << expected_yy
            << ", Izz=" << expected_zz << "\n";

  double error_xx = std::abs(I(X, X) - expected_xx);
  double error_yy = std::abs(I(Y, Y) - expected_yy);
  double error_zz = std::abs(I(Z, Z) - expected_zz);

  std::cout << "Errors: Ixx=" << error_xx << ", Iyy=" << error_yy
            << ", Izz=" << error_zz << "\n";

  bool passed = (error_xx < 1e-10 && error_yy < 1e-10 && error_zz < 1e-10);
  std::cout << "Result: " << (passed ? "PASS" : "FAIL") << "\n";
}

void testRegularTetrahedron()
{
  std::cout << "\n=== Test: Regular Tetrahedron (edge=2.0) ===\n";

  double edgeLength = 2.0;
  Mesh tet = createRegularTetrahedron(edgeLength);
  double mass = 1.0;

  Mat3 I = computeInertiaTensorAboutCentroid(tet, mass);

  // For regular tetrahedron: Ixx = Iyy = Izz = m*L²/20
  double expected_diagonal = mass * edgeLength * edgeLength / 20.0;

  std::cout << "Computed inertia tensor:\n";
  I.print();

  std::cout << "\nExpected diagonal (m*L²/20): " << expected_diagonal << "\n";

  // Check diagonal elements are equal (symmetry)
  double avg_diagonal = (I(X, X) + I(Y, Y) + I(Z, Z)) / 3.0;
  double symmetry_error = std::max({std::abs(I(X, X) - avg_diagonal),
                                    std::abs(I(Y, Y) - avg_diagonal),
                                    std::abs(I(Z, Z) - avg_diagonal)});

  std::cout << "Average diagonal: " << avg_diagonal << "\n";
  std::cout << "Symmetry error: " << symmetry_error << "\n";

  bool passed = (symmetry_error < 1e-10);
  std::cout << "Result: "
            << (passed ? "PASS (diagonal elements equal)" : "FAIL") << "\n";
}

// =============================================================================
// Main
// =============================================================================

int main()
{
  std::cout << std::fixed << std::setprecision(12);

  std::cout << "============================================================\n";
  std::cout << "Mirtich Algorithm Tutorial - Inertia Tensor Computation\n";
  std::cout << "============================================================\n";

  testUnitCube();
  testRectangularBox();
  testRegularTetrahedron();

  std::cout
    << "\n============================================================\n";
  std::cout << "Tutorial complete. See README.md for algorithm explanation.\n";
  std::cout << "============================================================\n";

  return 0;
}
