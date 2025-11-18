#ifndef GEOMETRY_FACTORY_HPP
#define GEOMETRY_FACTORY_HPP

#include <vector>
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim
{

/**
 * @brief Factory class for creating common 3D geometric shapes
 *
 * Provides static methods to generate vertex lists for standard shapes.
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
   * @return Vector of 36 vertices (12 triangles × 3 vertices)
   */
  static std::vector<Coordinate> createCube(double size);

  /**
   * @brief Create a wireframe cube (edges only) centered at origin
   *
   * Creates vertices for the 12 edges of a cube, useful for line rendering.
   * Returns pairs of vertices representing each edge.
   *
   * @param size Side length of the cube
   * @return Vector of 24 vertices (12 edges × 2 vertices)
   */
  static std::vector<Coordinate> createCubeWireframe(double size);

private:
  /**
   * @brief Helper to create the 8 corner vertices of a cube
   * @param size Side length of the cube
   * @return Array of 8 corner coordinates
   */
  static std::array<Coordinate, 8> getCubeCorners(double size);
};

}  // namespace msd_sim

#endif  // GEOMETRY_FACTORY_HPP
