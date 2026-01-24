/**
 * @file example.cpp
 * @brief Standalone tutorial implementation of the Expanding Polytope Algorithm (EPA)
 *
 * This file demonstrates the EPA algorithm for computing collision contact information
 * (penetration depth, contact normal, contact point) from two colliding convex shapes.
 *
 * Production equivalent: msd-sim/src/Physics/EPA.hpp, EPA.cpp
 * The production version uses:
 * - Eigen-based Coordinate and CoordinateRate types
 * - Integration with AssetPhysical and ReferenceFrame for world-space transforms
 * - Robust simplex completion for edge cases
 * This tutorial version prioritizes readability and educational clarity.
 *
 * @ticket 0027a_expanding_polytope_algorithm
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

// ============================================================================
// Vec3: Simple 3D vector for tutorial clarity
// Production equivalent: msd_sim::Coordinate (Eigen::Vector3d wrapper)
// ============================================================================

struct Vec3
{
  double x, y, z;

  Vec3() : x{0}, y{0}, z{0} {}
  Vec3(double x_, double y_, double z_) : x{x_}, y{y_}, z{z_} {}

  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
  Vec3 operator-() const { return {-x, -y, -z}; }

  double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

  Vec3 cross(const Vec3& o) const
  {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }

  double norm() const { return std::sqrt(x * x + y * y + z * z); }

  Vec3 normalized() const
  {
    double n = norm();
    return n > 1e-10 ? *this / n : Vec3{0, 0, 0};
  }
};

std::ostream& operator<<(std::ostream& os, const Vec3& v)
{
  return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

// ============================================================================
// ConvexShape: Simple convex hull representation
// Production equivalent: msd_sim::ConvexHull (uses Qhull library)
// ============================================================================

struct ConvexShape
{
  std::vector<Vec3> vertices;

  /**
   * @brief Support function: find vertex furthest in given direction
   *
   * Mathematical definition:
   *   support(S, d) = argmax_{v ∈ S} (v · d)
   *
   * This is the fundamental operation for GJK and EPA algorithms.
   * Complexity: O(n) where n = number of vertices
   *
   * @param direction Direction to search (need not be normalized)
   * @return Vertex with maximum dot product with direction
   */
  Vec3 support(const Vec3& direction) const
  {
    double maxDot = -std::numeric_limits<double>::infinity();
    Vec3 result = vertices[0];

    for (const auto& v : vertices)
    {
      double d = v.dot(direction);
      if (d > maxDot)
      {
        maxDot = d;
        result = v;
      }
    }

    return result;
  }
};

/**
 * @brief Create a unit cube centered at the origin
 * @return ConvexShape with 8 vertices forming a cube from (-0.5,-0.5,-0.5) to (0.5,0.5,0.5)
 */
ConvexShape createUnitCube()
{
  ConvexShape cube;
  for (int i = 0; i < 8; ++i)
  {
    double x = (i & 1) ? 0.5 : -0.5;
    double y = (i & 2) ? 0.5 : -0.5;
    double z = (i & 4) ? 0.5 : -0.5;
    cube.vertices.push_back({x, y, z});
  }
  return cube;
}

/**
 * @brief Translate all vertices of a shape
 * @param shape Shape to translate
 * @param offset Translation vector
 * @return New shape with translated vertices
 */
ConvexShape translate(const ConvexShape& shape, const Vec3& offset)
{
  ConvexShape result;
  for (const auto& v : shape.vertices)
  {
    result.vertices.push_back(v + offset);
  }
  return result;
}

// ============================================================================
// Minkowski Difference Support Function
// ============================================================================

/**
 * @brief Compute support point on Minkowski difference A ⊖ B
 *
 * The Minkowski difference is defined as:
 *   A ⊖ B = { a - b | a ∈ A, b ∈ B }
 *
 * Key insight: We don't need to compute all points in A ⊖ B.
 * For any direction d:
 *   support(A ⊖ B, d) = support(A, d) - support(B, -d)
 *
 * This is the fundamental operation that makes GJK and EPA efficient.
 *
 * @param shapeA First convex shape
 * @param shapeB Second convex shape
 * @param direction Direction to search in Minkowski space
 * @return Point on Minkowski difference boundary furthest in direction
 */
Vec3 supportMinkowski(const ConvexShape& shapeA,
                      const ConvexShape& shapeB,
                      const Vec3& direction)
{
  // Support on A in direction d
  Vec3 supportA = shapeA.support(direction);

  // Support on B in direction -d (opposite direction)
  Vec3 supportB = shapeB.support(-direction);

  // Minkowski difference: A - B
  return supportA - supportB;
}

// ============================================================================
// GJK Algorithm (Simplified for EPA setup)
// Production equivalent: msd_sim::GJK
// ============================================================================

/**
 * @brief Build a tetrahedron containing the origin for overlapping shapes
 *
 * For tutorial purposes, we use a direct approach: build a tetrahedron
 * from support points in 4 directions that spans the Minkowski difference.
 * The production GJK is more sophisticated.
 *
 * @param shapeA First convex shape
 * @param shapeB Second convex shape
 * @param[out] simplex Tetrahedron vertices
 * @return true if shapes intersect (tetrahedron contains origin)
 */
bool gjkIntersects(const ConvexShape& shapeA,
                   const ConvexShape& shapeB,
                   std::vector<Vec3>& simplex)
{
  simplex.clear();

  // Get 4 support points in different directions to form a tetrahedron
  Vec3 dirs[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

  // First, check if shapes are separated in any cardinal direction
  for (int i = 0; i < 6; ++i)
  {
    Vec3 p = supportMinkowski(shapeA, shapeB, dirs[i]);
    // If support point in direction d has negative projection,
    // the shapes are separated in that direction
    if (p.dot(dirs[i]) < 0 && i % 2 == 0)
    {
      // Shapes separated - no collision
      return false;
    }
  }

  // Build tetrahedron using 4 linearly independent directions
  simplex.push_back(supportMinkowski(shapeA, shapeB, Vec3{1, 1, 1}));
  simplex.push_back(supportMinkowski(shapeA, shapeB, Vec3{-1, -1, 1}));
  simplex.push_back(supportMinkowski(shapeA, shapeB, Vec3{-1, 1, -1}));
  simplex.push_back(supportMinkowski(shapeA, shapeB, Vec3{1, -1, -1}));

  // Check if origin is inside the tetrahedron
  // For each face, check if origin is on the inside
  auto sameSign = [](double a, double b) { return (a >= 0) == (b >= 0); };

  Vec3 a = simplex[0], b = simplex[1], c = simplex[2], d = simplex[3];
  Vec3 origin{0, 0, 0};

  // Compute signed volumes
  auto signedVolume = [](const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4) {
    Vec3 v1 = p2 - p1, v2 = p3 - p1, v3 = p4 - p1;
    return v1.dot(v2.cross(v3));
  };

  double d0 = signedVolume(a, b, c, d);
  double d1 = signedVolume(origin, b, c, d);
  double d2 = signedVolume(a, origin, c, d);
  double d3 = signedVolume(a, b, origin, d);
  double d4 = signedVolume(a, b, c, origin);

  // Origin is inside if all barycentric coordinates have the same sign
  bool inside = sameSign(d0, d1) && sameSign(d0, d2) && sameSign(d0, d3) && sameSign(d0, d4);

  return inside;
}

// ============================================================================
// EPA Data Structures
// ============================================================================

/**
 * @brief Triangular face in the EPA polytope
 *
 * Each face stores:
 * - Three vertex indices into the vertex array
 * - Outward-facing unit normal
 * - Distance from origin to the face plane
 */
struct EPAFace
{
  std::array<size_t, 3> vertices;  // Indices into polytope vertex array
  Vec3 normal;                     // Outward-facing unit normal
  double distance;                 // Distance from origin to face plane

  EPAFace() : vertices{0, 0, 0}, normal{}, distance{0} {}
  EPAFace(size_t v0, size_t v1, size_t v2, const Vec3& n, double d)
    : vertices{v0, v1, v2}, normal{n}, distance{d}
  {
  }
};

/**
 * @brief Edge in the polytope (for horizon construction)
 *
 * Edges are compared order-independently: (A,B) == (B,A)
 * This is important for identifying horizon edges.
 */
struct EPAEdge
{
  size_t v0, v1;

  EPAEdge(size_t a, size_t b) : v0{a}, v1{b} {}

  bool operator==(const EPAEdge& other) const
  {
    return (v0 == other.v0 && v1 == other.v1) ||
           (v0 == other.v1 && v1 == other.v0);
  }
};

/**
 * @brief Collision result containing contact information
 *
 * Note: No 'intersecting' boolean - collision state is conveyed
 * by optional return type in production code.
 */
struct CollisionResult
{
  Vec3 normal;           // Contact normal (points from A toward B)
  double penetrationDepth;  // How far objects overlap
  Vec3 contactPoint;     // Approximate contact location
};

// ============================================================================
// EPA Algorithm Implementation
// ============================================================================

/**
 * @brief Add a triangular face to the polytope
 *
 * Computes the face normal and distance from origin.
 * Ensures normal points outward (away from origin).
 *
 * @param faces Face array to add to
 * @param vertices Polytope vertex array
 * @param v0, v1, v2 Vertex indices for the new face
 * @param epsilon Tolerance for degenerate face detection
 */
void addFace(std::vector<EPAFace>& faces,
             const std::vector<Vec3>& vertices,
             size_t v0,
             size_t v1,
             size_t v2,
             double epsilon = 1e-6)
{
  const Vec3& a = vertices[v0];
  const Vec3& b = vertices[v1];
  const Vec3& c = vertices[v2];

  // Compute face normal via cross product of two edges
  // Normal = (B - A) × (C - A)
  Vec3 ab = b - a;
  Vec3 ac = c - a;
  Vec3 normal = ab.cross(ac);

  double normalLength = normal.norm();
  if (normalLength < epsilon)
  {
    // Degenerate face (coplanar vertices) - skip
    return;
  }

  normal = normal / normalLength;  // Normalize

  // Ensure normal points away from origin
  // If centroid · normal < 0, the normal points toward origin - flip it
  Vec3 centroid = (a + b + c) / 3.0;
  if (normal.dot(centroid) < 0.0)
  {
    normal = -normal;
    std::swap(v1, v2);  // Also flip vertex winding for consistency
  }

  // Distance from origin to face plane
  // For a plane with normal n passing through point p: distance = n · p
  double distance = normal.dot(a);

  faces.emplace_back(v0, v1, v2, normal, distance);
}

/**
 * @brief Find the face closest to the origin
 *
 * This face represents our current best approximation of the
 * closest point on the Minkowski boundary to the origin.
 *
 * @param faces Array of polytope faces
 * @return Index of closest face
 */
size_t findClosestFace(const std::vector<EPAFace>& faces)
{
  double minDistance = std::numeric_limits<double>::infinity();
  size_t closestIndex = 0;

  for (size_t i = 0; i < faces.size(); ++i)
  {
    if (faces[i].distance < minDistance)
    {
      minDistance = faces[i].distance;
      closestIndex = i;
    }
  }

  return closestIndex;
}

/**
 * @brief Check if a point is visible from a face
 *
 * A point is "visible" from a face if it lies on the positive
 * side of the face plane (in front of the face).
 *
 * @param face Face to check visibility from
 * @param vertices Polytope vertex array
 * @param point Point to test
 * @param epsilon Tolerance for plane test
 * @return true if point is visible from face
 */
bool isVisible(const EPAFace& face,
               const std::vector<Vec3>& vertices,
               const Vec3& point,
               double epsilon = 1e-6)
{
  Vec3 toPoint = point - vertices[face.vertices[0]];
  return face.normal.dot(toPoint) > epsilon;
}

/**
 * @brief Build horizon edges for polytope expansion
 *
 * When expanding the polytope with a new vertex, we need to:
 * 1. Find all faces visible from the new vertex
 * 2. Identify the "horizon" - edges shared by exactly one visible face
 * 3. Remove visible faces
 *
 * The horizon forms the silhouette of visible faces as seen from the new vertex.
 * New faces will connect the new vertex to each horizon edge.
 *
 * @param faces Polytope faces (will have visible faces removed)
 * @param vertices Polytope vertices
 * @param newVertex New vertex being added
 * @param epsilon Visibility tolerance
 * @return Vector of horizon edges
 */
std::vector<EPAEdge> buildHorizonEdges(std::vector<EPAFace>& faces,
                                        const std::vector<Vec3>& vertices,
                                        const Vec3& newVertex,
                                        double epsilon = 1e-6)
{
  std::vector<EPAEdge> horizon;
  std::vector<size_t> visibleFaceIndices;

  // Step 1: Find all faces visible from newVertex
  for (size_t i = 0; i < faces.size(); ++i)
  {
    if (isVisible(faces[i], vertices, newVertex, epsilon))
    {
      visibleFaceIndices.push_back(i);
    }
  }

  // Step 2: Collect all edges from visible faces
  std::vector<EPAEdge> edgeCandidates;
  for (size_t idx : visibleFaceIndices)
  {
    const auto& f = faces[idx];
    edgeCandidates.emplace_back(f.vertices[0], f.vertices[1]);
    edgeCandidates.emplace_back(f.vertices[1], f.vertices[2]);
    edgeCandidates.emplace_back(f.vertices[2], f.vertices[0]);
  }

  // Step 3: Horizon edges appear exactly once
  // (Edges shared by two visible faces appear twice and are internal)
  for (const auto& edge : edgeCandidates)
  {
    int count = std::count(edgeCandidates.begin(), edgeCandidates.end(), edge);
    if (count == 1)
    {
      horizon.push_back(edge);
    }
  }

  // Step 4: Remove visible faces using erase-remove idiom
  // Mark for deletion by setting distance to infinity
  for (size_t idx : visibleFaceIndices)
  {
    faces[idx].distance = std::numeric_limits<double>::infinity();
  }

  faces.erase(std::remove_if(faces.begin(),
                             faces.end(),
                             [](const EPAFace& f) {
                               return std::isinf(f.distance);
                             }),
              faces.end());

  return horizon;
}

/**
 * @brief Compute contact point from closest face
 *
 * Uses barycentric centroid of the face as an approximation.
 * More accurate methods project the origin onto the face.
 *
 * @param face Closest face
 * @param vertices Polytope vertices
 * @return Approximate contact point in Minkowski space
 */
Vec3 computeContactPoint(const EPAFace& face, const std::vector<Vec3>& vertices)
{
  const Vec3& a = vertices[face.vertices[0]];
  const Vec3& b = vertices[face.vertices[1]];
  const Vec3& c = vertices[face.vertices[2]];

  return (a + b + c) / 3.0;
}

/**
 * @brief Expanding Polytope Algorithm
 *
 * EPA computes detailed contact information from a GJK simplex:
 *
 * 1. Initialize polytope from GJK tetrahedron (4 vertices, 4 faces)
 * 2. Find face closest to origin
 * 3. Query support point in closest face's normal direction
 * 4. If new point is within tolerance, converged - extract contact info
 * 5. Otherwise, expand polytope:
 *    - Find faces visible from new point
 *    - Build horizon edges (silhouette of visible region)
 *    - Remove visible faces
 *    - Add new faces connecting new point to horizon
 * 6. Repeat until converged
 *
 * @param shapeA First convex shape
 * @param shapeB Second convex shape
 * @param simplex GJK terminating simplex (must contain 4 vertices)
 * @param epsilon Convergence tolerance
 * @param maxIterations Maximum expansion iterations
 * @return Collision result with penetration depth, normal, and contact point
 */
CollisionResult epa(const ConvexShape& shapeA,
                    const ConvexShape& shapeB,
                    const std::vector<Vec3>& simplex,
                    double epsilon = 1e-6,
                    int maxIterations = 64)
{
  // Validate input
  if (simplex.size() != 4)
  {
    throw std::runtime_error("EPA requires a 4-vertex simplex (tetrahedron)");
  }

  // Initialize polytope with simplex vertices
  std::vector<Vec3> vertices = simplex;
  std::vector<EPAFace> faces;

  // Create 4 initial faces from tetrahedron
  // Vertices: A=0, B=1, C=2, D=3
  addFace(faces, vertices, 0, 1, 2, epsilon);  // ABC
  addFace(faces, vertices, 0, 2, 3, epsilon);  // ACD
  addFace(faces, vertices, 0, 3, 1, epsilon);  // ADB
  addFace(faces, vertices, 1, 3, 2, epsilon);  // BDC

  std::cout << "EPA: Initial polytope with " << faces.size() << " faces\n";

  // Main expansion loop
  for (int iteration = 0; iteration < maxIterations; ++iteration)
  {
    // Find closest face to origin
    size_t closestIndex = findClosestFace(faces);
    const EPAFace& closestFace = faces[closestIndex];

    std::cout << "  Iteration " << iteration << ": closest face distance = "
              << closestFace.distance << "\n";

    // Query new support point in direction of closest face normal
    Vec3 newPoint = supportMinkowski(shapeA, shapeB, closestFace.normal);

    // Convergence check
    // If new point doesn't extend significantly beyond closest face, we're done
    double distanceToNewPoint = newPoint.dot(closestFace.normal);
    if (distanceToNewPoint - closestFace.distance < epsilon)
    {
      std::cout << "EPA: Converged after " << iteration << " iterations\n";

      CollisionResult result;
      result.normal = closestFace.normal;
      result.penetrationDepth = closestFace.distance;
      result.contactPoint = computeContactPoint(closestFace, vertices);
      return result;
    }

    // Expand polytope
    std::vector<EPAEdge> horizon =
        buildHorizonEdges(faces, vertices, newPoint, epsilon);

    // Add new vertex
    size_t newVertexIndex = vertices.size();
    vertices.push_back(newPoint);

    // Create new faces connecting new vertex to horizon edges
    for (const auto& edge : horizon)
    {
      addFace(faces, vertices, edge.v0, edge.v1, newVertexIndex, epsilon);
    }

    std::cout << "  Added " << horizon.size() << " new faces, total "
              << faces.size() << " faces\n";
  }

  throw std::runtime_error("EPA did not converge within max iterations");
}

// ============================================================================
// Main: Test Cases
// ============================================================================

int main()
{
  std::cout << "=== Expanding Polytope Algorithm (EPA) Tutorial ===\n\n";

  // Test Case 1: Two overlapping cubes along X-axis
  std::cout << "Test 1: Overlapping cubes (X-axis penetration)\n";
  std::cout << "------------------------------------------------\n";

  ConvexShape cubeA = createUnitCube();
  ConvexShape cubeB = translate(createUnitCube(), Vec3{0.7, 0.0, 0.0});

  std::vector<Vec3> simplex1;
  if (gjkIntersects(cubeA, cubeB, simplex1))
  {
    std::cout << "GJK: Collision detected, simplex has " << simplex1.size()
              << " vertices\n";

    try
    {
      CollisionResult result = epa(cubeA, cubeB, simplex1);
      std::cout << "\nEPA Result:\n";
      std::cout << "  Penetration depth: " << result.penetrationDepth << " m\n";
      std::cout << "  Contact normal: " << result.normal << "\n";
      std::cout << "  Contact point: " << result.contactPoint << "\n";

      // Expected: penetration ~0.3, normal ~(1, 0, 0)
      double expectedDepth = 0.3;
      std::cout << "  Expected depth: ~" << expectedDepth << "\n";
      std::cout << "  Depth error: " << std::abs(result.penetrationDepth - expectedDepth)
                << "\n";
    }
    catch (const std::exception& e)
    {
      std::cerr << "EPA Error: " << e.what() << "\n";
    }
  }
  else
  {
    std::cout << "GJK: No collision (unexpected!)\n";
  }

  std::cout << "\n";

  // Test Case 2: Cubes overlapping along Y-axis
  std::cout << "Test 2: Overlapping cubes (Y-axis penetration)\n";
  std::cout << "------------------------------------------------\n";

  ConvexShape cubeC = createUnitCube();
  ConvexShape cubeD = translate(createUnitCube(), Vec3{0.0, 0.6, 0.0});

  std::vector<Vec3> simplex2;
  if (gjkIntersects(cubeC, cubeD, simplex2))
  {
    std::cout << "GJK: Collision detected\n";

    try
    {
      CollisionResult result = epa(cubeC, cubeD, simplex2);
      std::cout << "\nEPA Result:\n";
      std::cout << "  Penetration depth: " << result.penetrationDepth << " m\n";
      std::cout << "  Contact normal: " << result.normal << "\n";

      // Expected: penetration ~0.4, normal ~(0, 1, 0) or (0, -1, 0)
      double expectedDepth = 0.4;
      std::cout << "  Expected depth: ~" << expectedDepth << "\n";
    }
    catch (const std::exception& e)
    {
      std::cerr << "EPA Error: " << e.what() << "\n";
    }
  }

  std::cout << "\n";

  // Test Case 3: Non-overlapping cubes
  std::cout << "Test 3: Non-overlapping cubes (no collision)\n";
  std::cout << "---------------------------------------------\n";

  ConvexShape cubeE = createUnitCube();
  ConvexShape cubeF = translate(createUnitCube(), Vec3{2.0, 0.0, 0.0});

  std::vector<Vec3> simplex3;
  if (gjkIntersects(cubeE, cubeF, simplex3))
  {
    std::cout << "GJK: Collision detected (unexpected!)\n";
  }
  else
  {
    std::cout << "GJK: No collision (correct)\n";
    std::cout << "EPA not invoked - only runs when collision detected\n";
  }

  std::cout << "\n=== Tutorial Complete ===\n";

  return 0;
}
