#ifndef MSD_SIM_PHYSICS_CONVEX_HULL_HPP
#define MSD_SIM_PHYSICS_CONVEX_HULL_HPP

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "msd-assets/src/Geometry.hpp"
#include "msd-assets/src/GeometryTraits.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/DataTypes/Facet.hpp"

// Forward declare Qhull C API types
extern "C"
{
  // NOLINTNEXTLINE(readability-identifier-naming)
  struct qhT;
#include <libqhull_r/geom_r.h>
#include <libqhull_r/libqhull_r.h>
}

namespace msd_sim
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
    Coordinate min;  // Minimum corner
    Coordinate max;  // Maximum corner
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

  // Template implementation must be in header
  template <msd_assets::IsVector3 VectorType>
  explicit ConvexHull(const std::vector<VectorType>& points) : ConvexHull()
  {
    if (points.empty())
    {
      throw std::runtime_error(
        "Cannot create convex hull from empty point set");
    }
    computeHull(points);
  }

  /**
   * @brief Create convex hull from msd_assets::Geometry object.
   *
   * Extracts all vertices from the geometry and computes their convex hull.
   *
   * @param geometry Geometry object to create hull from
   * @return ConvexHull of the geometry's vertices
   * @throws std::runtime_error if geometry is empty or degenerate
   */
  explicit ConvexHull(const msd_assets::CollisionGeometry& geometry);

  /**
   * @brief Get all vertices of the convex hull.
   *
   * Returns only the vertices that form the hull boundary (not interior
   * points).
   *
   * @return Vector of hull vertex coordinates
   */
  [[nodiscard]] const std::vector<Coordinate>& getVertices() const;

  /**
   * @brief Get all facets of the convex hull.
   *
   * Each facet is a triangle with vertex indices and normal vector.
   *
   * @return Vector of triangular facets
   */
  [[nodiscard]] const std::vector<Facet>& getFacets() const;

  /**
   * @brief Get the facet most aligned with a given direction.
   * @param normal Direction to compare against
   * @return Reference to the facet with greatest dot product
   */
  [[nodiscard]] const Facet& getFacetAlignedWith(
    const msd_sim::Vector3D& normal) const;

  /**
   * @brief Get all facets aligned with a given direction within tolerance.
   *
   * Returns all facets whose alignment (dot product with normal) is within
   * tolerance of the maximum alignment. Useful for contact manifold generation
   * when multiple coplanar facets may contribute to the contact.
   *
   * @param normal Direction to compare against
   * @param tolerance How close to max alignment to include (default: 1e-6)
   * @return Vector of references to aligned facets
   */
  [[nodiscard]] std::vector<std::reference_wrapper<const Facet>>
  getFacetsAlignedWith(const msd_sim::Vector3D& normal,
                       double tolerance = 1e-9) const;

  /**
   * @brief Get the number of vertices in the hull.
   * @return Number of vertices
   */
  [[nodiscard]] size_t getVertexCount() const;

  /**
   * @brief Get the number of facets in the hull.
   * @return Number of triangular facets
   */
  [[nodiscard]] size_t getFacetCount() const;

  /**
   * @brief Get the volume enclosed by the convex hull.
   *
   * Volume is computed by Qhull during hull construction.
   *
   * @return Volume in cubic units
   */
  [[nodiscard]] double getVolume() const;

  /**
   * @brief Get the surface area of the convex hull.
   *
   * Surface area is computed by Qhull during hull construction.
   *
   * @return Surface area in square units
   */
  [[nodiscard]] double getSurfaceArea() const;

  /**
   * @brief Get the geometric centroid of the convex hull.
   *
   * The centroid is the center of mass assuming uniform density.
   * Computed during hull construction using tetrahedron decomposition.
   *
   * @return Centroid coordinate
   */
  [[nodiscard]] Coordinate getCentroid() const;

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
  [[nodiscard]] bool contains(const Coordinate& point,
                              double epsilon = 1e-6) const;

  /**
   * @brief Compute signed distance from a point to the hull surface.
   *
   * Negative distance means the point is inside the hull.
   * Positive distance means the point is outside.
   *
   * @param point Point to compute distance from
   * @return Signed distance to hull surface
   */
  [[nodiscard]] double signedDistance(const Coordinate& point) const;

  /**
   * @brief Get the axis-aligned bounding box of the convex hull.
   *
   * The bounding box is computed directly by Qhull during hull construction.
   *
   * @return Bounding box containing minimum and maximum corners
   */
  [[nodiscard]] BoundingBox getBoundingBox() const;

  /**
   * @brief Check if the hull is valid (non-degenerate).
   * @return true if hull has volume > 0, false otherwise
   */
  [[nodiscard]] bool isValid() const;

  /**
   * @brief Represents an edge of the convex hull as a pair of vertex
   * coordinates.
   * @ticket 0040c_edge_contact_manifold
   */
  struct Edge
  {
    Coordinate start;
    Coordinate end;
  };

  /**
   * @brief Find the edge of the convex hull closest to a given point in
   * hull-local space.
   *
   * Iterates all unique edges (vertex pairs from facet definitions) and
   * returns the one with minimum distance to the query point.
   *
   * @param point Query point in hull-local space
   * @return Edge with start and end coordinates in hull-local space
   * @ticket 0040c_edge_contact_manifold
   */
  [[nodiscard]] Edge findClosestEdge(const Coordinate& point) const;

private:
  std::vector<Coordinate> vertices_;  // Hull boundary vertices
  std::vector<Facet> facets_;         // Triangular facets with normals
  double volume_;                     // Volume computed by Qhull
  double surfaceArea_;                // Surface area computed by Qhull
  Coordinate boundingBoxMin_;         // Bounding box minimum from Qhull
  Coordinate boundingBoxMax_;         // Bounding box maximum from Qhull
  Coordinate centroid_;               // The centroid of the convex hull

  /**
   * @brief Internal template method to compute the convex hull using Qhull.
   *
   * Accepts any vector-like type that provides x(), y(), z() accessors,
   * such as msd_sim::Vector3D or msd_sim::Coordinate.
   *
   * @tparam VectorType Type with x(), y(), z() methods returning doubles
   * @param points Input point cloud
   * @throws std::runtime_error if Qhull computation fails
   */
  template <msd_assets::IsVector3 VectorType>
  void computeHull(const std::vector<VectorType>& points)
  {
    if (points.size() < 4)
    {
      throw std::runtime_error(
        "Cannot create 3D convex hull from fewer than 4 points");
    }

    // Convert points to Qhull format (flat array of doubles)
    std::vector<double> qhullPoints;
    qhullPoints.reserve(points.size() * 3);

    for (const auto& point : points)
    {
      qhullPoints.push_back(point.x());
      qhullPoints.push_back(point.y());
      qhullPoints.push_back(point.z());
    }

    // Initialize thread-local Qhull state
    qhT qhQh;
    qhT* qh = &qhQh;

    QHULL_LIB_CHECK
    qh_zero(qh, stderr);

    try
    {
      // Run Qhull with reentrant API
      // "qhull" = required command prefix
      // "Qt" = triangulated output (ensures all facets are triangles)
      // "Pp" = suppress progress messages
      char options[] = "qhull Qt Pp";
      const int exitcode =
        qh_new_qhull(qh,
                     3,                                // dimension
                     static_cast<int>(points.size()),  // numpoints
                     qhullPoints.data(),               // points array
                     False,    // ismalloc (we manage memory)
                     options,  // options (non-const for qhull's parsing)
                     stderr,   // outfile
                     stderr);  // errfile

      if (exitcode != 0)
      {
        int curlong{};
        int totlong{};
        qh_freeqhull(qh, !qh_ALL);
        qh_memfreeshort(qh, &curlong, &totlong);

        std::ostringstream oss;
        oss << "Qhull failed with exit code " << exitcode;
        throw std::runtime_error(oss.str());
      }

      // Calculate volume and surface area
      qh_getarea(qh, qh->facet_list);
      volume_ = qh->totvol;
      surfaceArea_ = qh->totarea;

      // Extract hull data (vertices and facets)
      extractHullData(qh);

      // Compute bounding box from extracted vertices
      if (!vertices_.empty())
      {
        boundingBoxMin_ = vertices_[0];
        boundingBoxMax_ = vertices_[0];
        for (const auto& v : vertices_)
        {
          boundingBoxMin_ = Coordinate{std::min(boundingBoxMin_.x(), v.x()),
                                       std::min(boundingBoxMin_.y(), v.y()),
                                       std::min(boundingBoxMin_.z(), v.z())};
          boundingBoxMax_ = Coordinate{std::max(boundingBoxMax_.x(), v.x()),
                                       std::max(boundingBoxMax_.y(), v.y()),
                                       std::max(boundingBoxMax_.z(), v.z())};
        }
      }

      // Clean up Qhull
      int curlong{};
      int totlong{};
      qh_freeqhull(qh, !qh_ALL);
      qh_memfreeshort(qh, &curlong, &totlong);

      // Compute centroid after hull data is extracted
      computeCentroid();
    }
    catch (const std::exception& e)
    {
      // Ensure cleanup on exception
      int curlong{};
      int totlong{};
      qh_freeqhull(qh, !qh_ALL);
      qh_memfreeshort(qh, &curlong, &totlong);

      std::ostringstream oss;
      oss << "Failed to compute convex hull: " << e.what();
      throw std::runtime_error(oss.str());
    }
  }
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


}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONVEX_HULL_HPP
