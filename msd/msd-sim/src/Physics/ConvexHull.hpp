#ifndef MSD_SIM_PHYSICS_CONVEX_HULL_HPP
#define MSD_SIM_PHYSICS_CONVEX_HULL_HPP

#include "msd-sim/src/Environment/Coordinate.hpp"
#include <vector>
#include <array>
#include <memory>
#include <optional>

// Forward declare Qhull classes to avoid header pollution
namespace orgQhull {
class Qhull;
}

namespace msd_assets {
class Geometry;
}

namespace msd_sim {

/**
 * @brief Represents a 3D convex hull computed using the Qhull library.
 *
 * This class wraps Qhull functionality to compute and query convex hulls
 * from point clouds or geometry objects. The convex hull is the smallest
 * convex polyhedron that contains all input points.
 *
 * Key features:
 * - Construct from point clouds or msd_assets::Geometry objects
 * - Query hull properties (volume, centroid, surface area)
 * - Point containment testing
 * - Access to hull vertices and facets
 * - Support for geometric intersection operations
 */
class ConvexHull
{
public:
  /**
   * @brief Represents a triangular facet of the convex hull.
   *
   * Each facet stores indices into the hull's vertex array,
   * along with the outward-facing normal vector.
   */
  struct Facet
  {
    std::array<size_t, 3> vertexIndices;  // Indices of triangle vertices
    Coordinate normal;                     // Outward-facing unit normal
    float offset;                          // Distance from origin (for half-space)
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
  explicit ConvexHull(const std::vector<Coordinate>& points);

  /**
   * @brief Copy constructor.
   */
  ConvexHull(const ConvexHull& other);

  /**
   * @brief Move constructor.
   */
  ConvexHull(ConvexHull&& other) noexcept;

  /**
   * @brief Copy assignment operator.
   */
  ConvexHull& operator=(const ConvexHull& other);

  /**
   * @brief Move assignment operator.
   */
  ConvexHull& operator=(ConvexHull&& other) noexcept;

  /**
   * @brief Destructor.
   */
  ~ConvexHull();

  /**
   * @brief Create convex hull from msd_assets::Geometry object.
   *
   * Extracts all vertices from the geometry and computes their convex hull.
   *
   * @param geometry Geometry object to create hull from
   * @return ConvexHull of the geometry's vertices
   * @throws std::runtime_error if geometry is empty or degenerate
   */
  static ConvexHull fromGeometry(const msd_assets::Geometry& geometry);

  /**
   * @brief Create convex hull from a point cloud.
   *
   * Alternative factory method for clarity.
   *
   * @param points Vector of 3D coordinates
   * @return ConvexHull of the points
   */
  static ConvexHull fromPoints(const std::vector<Coordinate>& points);

  /**
   * @brief Get all vertices of the convex hull.
   *
   * Returns only the vertices that form the hull boundary (not interior points).
   *
   * @return Vector of hull vertex coordinates
   */
  const std::vector<Coordinate>& getVertices() const;

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
   * @brief Compute the volume enclosed by the convex hull.
   * @return Volume in cubic units
   */
  float volume() const;

  /**
   * @brief Compute the surface area of the convex hull.
   * @return Surface area in square units
   */
  float surfaceArea() const;

  /**
   * @brief Compute the geometric centroid of the convex hull.
   *
   * The centroid is the center of mass assuming uniform density.
   *
   * @return Centroid coordinate
   */
  Coordinate centroid() const;

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
  bool contains(const Coordinate& point, float epsilon = 1e-6f) const;

  /**
   * @brief Compute signed distance from a point to the hull surface.
   *
   * Negative distance means the point is inside the hull.
   * Positive distance means the point is outside.
   *
   * @param point Point to compute distance from
   * @return Signed distance to hull surface
   */
  float signedDistance(const Coordinate& point) const;

  /**
   * @brief Get the bounding box of the convex hull.
   *
   * @param min Output: minimum corner of bounding box
   * @param max Output: maximum corner of bounding box
   */
  void getBoundingBox(Coordinate& min, Coordinate& max) const;

  /**
   * @brief Check if the hull is valid (non-degenerate).
   * @return true if hull has volume > 0, false otherwise
   */
  bool isValid() const;

  /**
   * @brief Get the number of input points used to create this hull.
   * @return Number of original input points
   */
  size_t getInputPointCount() const;

private:
  std::vector<Coordinate> vertices_;     // Hull boundary vertices
  std::vector<Facet> facets_;            // Triangular facets with normals
  mutable float cachedVolume_;           // Cached volume (-1 if not computed)
  mutable float cachedSurfaceArea_;      // Cached surface area (-1 if not computed)
  mutable Coordinate cachedCentroid_;    // Cached centroid
  mutable bool centroidValid_;           // Whether cached centroid is valid
  size_t inputPointCount_;               // Number of original input points

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
   * @param qhull Qhull object with computed hull
   */
  void extractHullData(const orgQhull::Qhull& qhull);

  /**
   * @brief Invalidate cached values (called when hull changes).
   */
  void invalidateCache();
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONVEX_HULL_HPP
