#ifndef GEOMETRY_FACTORY_HPP
#define GEOMETRY_FACTORY_HPP

#include <Eigen/Dense>
#include <array>
#include <vector>
#include "msd-assets/src/Geometry.hpp"
#include "msd-assets/src/GeometryTraits.hpp"


namespace msd_assets
{

/**
 * @brief Factory class for creating common 3D geometric shapes
 *
 * Provides static template methods to generate VisualGeometry or CollisionGeometry
 * objects for standard shapes. All geometries are triangulated and centered at
 * the origin.
 *
 * Usage:
 *   auto cube = GeometryFactory::createCube<VisualGeometry>(1.0);
 *   auto collisionCube = GeometryFactory::createCube<CollisionGeometry>(1.0);
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
   * @tparam GeometryType Either VisualGeometry or CollisionGeometry
   * @param size Side length of the cube
   * @return Geometry object containing 36 vertices (12 triangles × 3 vertices)
   */
  template <IsGeometry GeometryType = VisualGeometry>
  static GeometryType createCube(double size)
  {

    auto corners = getCubeCorners(size);
    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(36);  // 6 faces × 2 triangles × 3 vertices

    // Each face is composed of 2 triangles
    // Vertices are ordered counter-clockwise for front-facing

    // Front face (z = -half)
    vertices.push_back(corners[0]);  // Triangle 1
    vertices.push_back(corners[1]);
    vertices.push_back(corners[2]);
    vertices.push_back(corners[0]);  // Triangle 2
    vertices.push_back(corners[2]);
    vertices.push_back(corners[3]);

    // Back face (z = +half)
    vertices.push_back(corners[5]);  // Triangle 1
    vertices.push_back(corners[4]);
    vertices.push_back(corners[7]);
    vertices.push_back(corners[5]);  // Triangle 2
    vertices.push_back(corners[7]);
    vertices.push_back(corners[6]);

    // Left face (x = -half)
    vertices.push_back(corners[4]);  // Triangle 1
    vertices.push_back(corners[0]);
    vertices.push_back(corners[3]);
    vertices.push_back(corners[4]);  // Triangle 2
    vertices.push_back(corners[3]);
    vertices.push_back(corners[7]);

    // Right face (x = +half)
    vertices.push_back(corners[1]);  // Triangle 1
    vertices.push_back(corners[5]);
    vertices.push_back(corners[6]);
    vertices.push_back(corners[1]);  // Triangle 2
    vertices.push_back(corners[6]);
    vertices.push_back(corners[2]);

    // Top face (y = -half)
    vertices.push_back(corners[4]);  // Triangle 1
    vertices.push_back(corners[5]);
    vertices.push_back(corners[1]);
    vertices.push_back(corners[4]);  // Triangle 2
    vertices.push_back(corners[1]);
    vertices.push_back(corners[0]);

    // Bottom face (y = +half)
    vertices.push_back(corners[3]);  // Triangle 1
    vertices.push_back(corners[2]);
    vertices.push_back(corners[6]);
    vertices.push_back(corners[3]);  // Triangle 2
    vertices.push_back(corners[6]);
    vertices.push_back(corners[7]);

    return GeometryType{vertices};
  }

  /**
   * @brief Create a wireframe cube (edges only) centered at origin
   *
   * Creates vertices for the 12 edges of a cube, useful for line rendering.
   * Returns pairs of vertices representing each edge.
   *
   * @tparam GeometryType Either VisualGeometry or CollisionGeometry
   * @param size Side length of the cube
   * @return Geometry object containing 24 vertices (12 edges × 2 vertices)
   */
  template <IsGeometry GeometryType = VisualGeometry>
  static GeometryType createCubeWireframe(double size)
  {

    auto corners = getCubeCorners(size);
    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(24);  // 12 edges × 2 vertices

    // Front face edges
    vertices.push_back(corners[0]);
    vertices.push_back(corners[1]);
    vertices.push_back(corners[1]);
    vertices.push_back(corners[2]);
    vertices.push_back(corners[2]);
    vertices.push_back(corners[3]);
    vertices.push_back(corners[3]);
    vertices.push_back(corners[0]);

    // Back face edges
    vertices.push_back(corners[4]);
    vertices.push_back(corners[5]);
    vertices.push_back(corners[5]);
    vertices.push_back(corners[6]);
    vertices.push_back(corners[6]);
    vertices.push_back(corners[7]);
    vertices.push_back(corners[7]);
    vertices.push_back(corners[4]);

    // Connecting edges between front and back
    vertices.push_back(corners[0]);
    vertices.push_back(corners[4]);
    vertices.push_back(corners[1]);
    vertices.push_back(corners[5]);
    vertices.push_back(corners[2]);
    vertices.push_back(corners[6]);
    vertices.push_back(corners[3]);
    vertices.push_back(corners[7]);

    return GeometryType{vertices};
  }

  /**
   * @brief Create a pyramid centered at origin
   *
   * Creates a pyramid with a square base and apex, triangulated.
   *
   * @tparam GeometryType Either VisualGeometry or CollisionGeometry
   * @param baseSize Side length of the square base
   * @param height Height of the pyramid
   * @return Geometry object containing pyramid vertices
   */
  template <IsGeometry GeometryType = VisualGeometry>
  static GeometryType createPyramid(double baseSize, double height)
  {

    float half = static_cast<float>(baseSize) / 2.0f;
    float halfHeight = static_cast<float>(height) / 2.0f;
    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(
      18);  // 4 side faces (triangles) + 2 base triangles = 6 triangles

    // Base corners (y = -height/2)
    Eigen::Vector3d base_fl{-half, -halfHeight, -half};  // front-left
    Eigen::Vector3d base_fr{half, -halfHeight, -half};   // front-right
    Eigen::Vector3d base_br{half, -halfHeight, half};    // back-right
    Eigen::Vector3d base_bl{-half, -halfHeight, half};   // back-left

    // Apex (top of pyramid)
    Eigen::Vector3d apex{0.0f, halfHeight, 0.0f};

    // Front face
    vertices.push_back(base_fl);
    vertices.push_back(base_fr);
    vertices.push_back(apex);

    // Right face
    vertices.push_back(base_fr);
    vertices.push_back(base_br);
    vertices.push_back(apex);

    // Back face
    vertices.push_back(base_br);
    vertices.push_back(base_bl);
    vertices.push_back(apex);

    // Left face
    vertices.push_back(base_bl);
    vertices.push_back(base_fl);
    vertices.push_back(apex);

    // Base bottom (2 triangles)
    vertices.push_back(base_fl);
    vertices.push_back(base_br);
    vertices.push_back(base_fr);

    vertices.push_back(base_fl);
    vertices.push_back(base_bl);
    vertices.push_back(base_br);

    return GeometryType{vertices};
  }

private:
  /**
   * @brief Helper to create the 8 corner vertices of a cube
   * @param size Side length of the cube
   * @return Array of 8 corner coordinates
   */
  static std::array<Eigen::Vector3d, 8> getCubeCorners(double size);
};

}  // namespace msd_assets

#endif  // GEOMETRY_FACTORY_HPP
