#ifndef MSD_SIM_PHYSICS_CONVEX_HULL_HPP
#define MSD_SIM_PHYSICS_CONVEX_HULL_HPP

#include <array>
#include <memory>
#include <optional>
#include <vector>

#include "msd-sim/src/Environment/Coordinate.hpp"

// Forward declare Qhull C API types
extern "C"
{
  struct qhT;
}

namespace msd_assets
{

/**
 * @brief Represents a 3D convex hull for collision detection.
 *
 * This class wraps Qhull functionality to compute convex hulls from point
 * clouds or geometry objects. It provides the minimal data required for
 * collision detection algorithms (GJK, EPA, etc.).
 *
 * Key features:
 * - Construct from point clouds or msd_assets::Geometry objects
 * - Access to hull vertices and facets
 * - Point containment testing
 * - Support for geometric intersection operations
 * - Axis-aligned bounding box for broad-phase collision detection
 *
 * Note: For inertial properties (mass, centroid, moment of inertia), use
 * AssetInertial which contains a ConvexHull along with physics properties.
 */
class ConvexHull
{
public:
  /**
   * @brief Axis-aligned bounding box.
   */
  struct BoundingBox
  {
    msd_sim::Coordinate min;  // Minimum corner
    msd_sim::Coordinate max;  // Maximum corner
  };

  /**
   * @brief Represents a triangular facet of the convex hull.
   *
   * Each facet stores indices into the hull's vertex array,
   * along with the outward-facing normal vector.
   */
  struct Facet
  {
    std::array<size_t, 3> vertexIndices;  // Indices of triangle vertices
    msd_sim::Coordinate normal;           // Outward-facing unit normal
    double offset;  // Distance from origin (for half-space)
  };

  /**
   * @brief Default constructor - creates an empty hull.
   */
  ConvexHull();

  /**
   * @brief Construct convex hull from a point cloud.
   *
   * Computes the convex hull of the given points using Qhull.
   * Duplicate and interior points are automatically removed.
   *
   * @param points Vector of 3D coordinates
   * @throws std::runtime_error if points are degenerate or Qhull fails
   */
  explicit ConvexHull(const std::vector<msd_sim::Coordinate>& points);

  /**
   * @brief Create convex hull from msd_assets::Geometry object.
   *
   * Extracts all vertices from the geometry and computes their convex hull.
   *
   * @param geometry Geometry object to create hull from
   * @return ConvexHull of the geometry's vertices
   * @throws std::runtime_error if geometry is empty or degenerate
   */
  explicit ConvexHull(const Geometry& geometry);

  /**
   * @brief Get all vertices of the convex hull.
   *
   * Returns only the vertices that form the hull boundary (not interior
   * points).
   *
   * @return Vector of hull vertex coordinates
   */
  const std::vector<msd_sim::Coordinate>& getVertices() const;

  /**
   * @brief Get all facets of the convex hull.
   *
   * Each facet is a triangle with vertex indices and normal vector.
   *
   * @return Vector of triangular facets
   */
  const std::vector<Facet>& getFacets() const;

  /**
   * @brief Get the number of vertices in the hull.
   * @return Number of vertices
   */
  size_t getVertexCount() const;

  /**
   * @brief Get the number of facets in the hull.
   * @return Number of triangular facets
   */
  size_t getFacetCount() const;

  /**
   * @brief Get the volume enclosed by the convex hull.
   *
   * Volume is computed by Qhull during hull construction.
   *
   * @return Volume in cubic units
   */
  double getVolume() const;

  /**
   * @brief Get the surface area of the convex hull.
   *
   * Surface area is computed by Qhull during hull construction.
   *
   * @return Surface area in square units
   */
  double getSurfaceArea() const;

  /**
   * @brief Get the geometric centroid of the convex hull.
   *
   * The centroid is the center of mass assuming uniform density.
   * Computed during hull construction using tetrahedron decomposition.
   *
   * @return Centroid coordinate
   */
  msd_sim::Coordinate getCentroid() const;

  /**
   * @brief Test if a point is contained within the convex hull.
   *
   * Uses the half-space representation: a point is inside if it's
   * on the negative side of all facet planes.
   *
   * @param point Point to test
   * @param epsilon Tolerance for numerical precision (default: 1e-6)
   * @return true if point is inside or on the boundary, false otherwise
   */
  bool contains(const msd_sim::Coordinate& point, double epsilon = 1e-6) const;

  /**
   * @brief Compute signed distance from a point to the hull surface.
   *
   * Negative distance means the point is inside the hull.
   * Positive distance means the point is outside.
   *
   * @param point Point to compute distance from
   * @return Signed distance to hull surface
   */
  double signedDistance(const msd_sim::Coordinate& point) const;

  /**
   * @brief Get the axis-aligned bounding box of the convex hull.
   *
   * The bounding box is computed directly by Qhull during hull construction.
   *
   * @return Bounding box containing minimum and maximum corners
   */
  BoundingBox getBoundingBox() const;

  /**
   * @brief Check if the hull is valid (non-degenerate).
   * @return true if hull has volume > 0, false otherwise
   */
  bool isValid() const;

  /**
   * @brief Test if this hull intersects with another using GJK algorithm.
   *
   * Uses the Gilbert-Johnson-Keerthi (GJK) algorithm to efficiently detect
   * intersection between two convex hulls. This is typically faster than
   * brute-force vertex containment tests and works in all cases.
   *
   * @param other The other convex hull to test against
   * @param epsilon Numerical tolerance for termination (default: 1e-6)
   * @return true if the hulls intersect, false otherwise
   */
  bool intersects(const ConvexHull& other, double epsilon = 1e-6) const;

private:
  std::vector<msd_sim::Coordinate> vertices_;  // Hull boundary vertices
  std::vector<Facet> facets_;                  // Triangular facets with normals
  double volume_;                              // Volume computed by Qhull
  double surfaceArea_;                         // Surface area computed by Qhull
  msd_sim::Coordinate boundingBoxMin_;  // Bounding box minimum from Qhull
  msd_sim::Coordinate boundingBoxMax_;  // Bounding box maximum from Qhull
  msd_sim::Coordinate centroid_;        // The centroid of the convex hull

  /**
   * @brief Internal method to compute the convex hull using Qhull.
   *
   * @param points Input point cloud
   * @throws std::runtime_error if Qhull computation fails
   */
  void computeHull(const std::vector<Coordinate>& points);

  /**
   * @brief Extract vertices and facets from Qhull result.
   *
   * @param qh Qhull state structure
   */
  void extractHullData(qhT* qh);


  /**
   * @brief Compute and cache the geometric centroid.
   *
   * The centroid is the center of mass assuming uniform density.
   * Uses tetrahedron decomposition for accurate volume-weighted calculation.
   * Called during hull construction.
   */
  void computeCentroid();
};

}  // namespace msd_assets

#endif  // MSD_SIM_PHYSICS_CONVEX_HULL_HPP
