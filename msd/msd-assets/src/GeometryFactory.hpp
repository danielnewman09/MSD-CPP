#ifndef GEOMETRY_FACTORY_HPP
#define GEOMETRY_FACTORY_HPP

#include <Eigen/Dense>
#include <array>
#include <vector>
#include "msd-assets/src/Geometry.hpp"
#include "msd-assets/src/GeometryTraits.hpp"
#include "msd-transfer/src/MeshRecord.hpp"


namespace msd_assets
{

/**
 * @brief Factory class for creating common 3D geometric shapes
 *
 * Provides static template methods to generate MeshRecord or
 * CollisionMeshRecord objects for standard shapes. All geometries are
 * triangulated and centered at the origin.
 *
 * Usage:
 *   auto cube = GeometryFactory::createCube<msd_transfer::MeshRecord>(1.0);
 *   auto collisionCube =
 * GeometryFactory::createCube<msd_transfer::CollisionMeshRecord>(1.0);
 */
class GeometryFactory
{
public:
  /**
   * @brief Create a cube centered at origin
   *
   * Creates a cube with side length 'size', triangulated into 12 triangles
   * (2 per face, 6 faces). Vertices are ordered for proper rendering.
   *
   * The cube extends from -size/2 to +size/2 in all dimensions.
   *
   * @tparam T Either MeshRecord or CollisionMeshRecord
   * @param size Side length of the cube
   * @return MeshRecord object containing 36 vertices (12 triangles × 3
   * vertices)
   */
  static msd_transfer::MeshRecord createCube(double size);

  /**
   * @brief Create a wireframe cube (edges only) centered at origin
   *
   * Creates vertices for the 12 edges of a cube, useful for line rendering.
   * Returns pairs of vertices representing each edge.
   *
   * @tparam T Either MeshRecord or CollisionMeshRecord
   * @param size Side length of the cube
   * @return MeshRecord object containing 24 vertices (12 edges × 2 vertices)
   */
  static msd_transfer::MeshRecord createCubeWireframe(double size);

  /**
   * @brief Create a pyramid centered at origin
   *
   * Creates a pyramid with a square base and apex, triangulated.
   *
   * @tparam T Either MeshRecord or CollisionMeshRecord
   * @param baseSize Side length of the square base
   * @param height Height of the pyramid
   * @return MeshRecord object containing pyramid vertices
   */
  static msd_transfer::MeshRecord createPyramid(double baseSize, double height);

private:
  /**
   * @brief Helper to create the 8 corner vertices of a cube
   * @param size Side length of the cube
   * @return Array of 8 corner coordinates
   */
  static std::array<Eigen::Vector3d, 8> getCubeCorners(double size);

  /**
   * @brief Convert vertices to CollisionMeshRecord
   *
   * Creates a CollisionMeshRecord with hull_data BLOB and bounding box from
   * vertices.
   *
   * @param vertices Vector of 3D coordinates representing collision hull
   * @return CollisionMeshRecord with populated hull_data and bounding volume
   */
  static msd_transfer::MeshRecord verticesToMeshRecord(
    const std::vector<Eigen::Vector3d>& vertices);
};

}  // namespace msd_assets

#endif  // GEOMETRY_FACTORY_HPP
