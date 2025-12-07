#ifndef GEOMETRY_FACTORY_HPP
#define GEOMETRY_FACTORY_HPP

#include <array>
#include <vector>
#include "msd-assets/src/Geometry.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_assets
{

/**
 * @brief Factory class for creating common 3D geometric shapes
 *
 * Provides static methods to generate Geometry objects for standard shapes.
 * All geometries are triangulated and centered at the origin.
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
   * @param size Side length of the cube
   * @return Geometry object containing 36 vertices (12 triangles × 3 vertices)
   */
  static Geometry createCube(double size);

  /**
   * @brief Create a wireframe cube (edges only) centered at origin
   *
   * Creates vertices for the 12 edges of a cube, useful for line rendering.
   * Returns pairs of vertices representing each edge.
   *
   * @param size Side length of the cube
   * @return Geometry object containing 24 vertices (12 edges × 2 vertices)
   */
  static Geometry createCubeWireframe(double size);

  /**
   * @brief Create a pyramid centered at origin
   *
   * Creates a pyramid with a square base and apex, triangulated.
   *
   * @param baseSize Side length of the square base
   * @param height Height of the pyramid
   * @return Geometry object containing pyramid vertices
   */
  static Geometry createPyramid(double baseSize, double height);

private:
  /**
   * @brief Helper to create the 8 corner vertices of a cube
   * @param size Side length of the cube
   * @return Array of 8 corner coordinates
   */
  static std::array<msd_sim::Coordinate, 8> getCubeCorners(double size);
};

}  // namespace msd_assets

#endif  // GEOMETRY_FACTORY_HPP
