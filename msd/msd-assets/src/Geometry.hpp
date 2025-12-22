#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <vector>
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_assets
{

struct Vertex
{
  float position[3];  // Position (x, y, z)
  float color[3];     // Color (r, g, b)
  float normal[3];    // Normal vector (x, y, z)
};


/**
 * @brief Simple 3D geometry container storing a collection of vertices
 *
 * This class represents pure geometry data - just a list of 3D coordinates.
 * It contains no rendering logic or dependencies on graphics APIs.
 * Rendering is handled by the msd-gui layer.
 *
 * Vertices are stored as triangulated mesh data, where each group of 3
 * vertices represents a triangle.
 */
class Geometry
{
public:
  /**
   * @brief Default constructor - creates empty geometry
   */
  Geometry() = default;

  /**
   * @brief Constructor with vertex list
   * @param vertices Vector of 3D coordinates representing the geometry
   */
  explicit Geometry(const std::vector<msd_sim::Coordinate>& vertices);

  /**
   * @brief Convert Geometry to Vertex vector with colors
   * @param r Red component (0.0-1.0)
   * @param g Green component (0.0-1.0)
   * @param b Blue component (0.0-1.0)
   * @return Vector of Vertex structs ready for GPU upload
   */
  std::vector<Vertex> toGUIVertices(float r = 1.0f,
                                    float g = 1.0f,
                                    float b = 1.0f);

  /**
   * @brief Get all vertices
   * @return Const reference to the vertex vector
   */
  const std::vector<msd_sim::Coordinate>& getVertices() const;

  /**
   * @brief Get number of vertices
   * @return Number of vertices in the geometry
   */
  size_t getVertexCount() const;

private:
  std::vector<msd_sim::Coordinate> vertices_;
};

}  // namespace msd_assets

#endif  // GEOMETRY_HPP
