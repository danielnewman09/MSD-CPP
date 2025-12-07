#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <vector>
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_assets
{

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
   * @brief Set all vertices
   * @param vertices Vector of 3D coordinates
   */
  void setVertices(const std::vector<msd_sim::Coordinate>& vertices);

  /**
   * @brief Add a single vertex to the geometry
   * @param vertex 3D coordinate to add
   */
  void addVertex(const msd_sim::Coordinate& vertex);

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

  /**
   * @brief Clear all vertices
   */
  void clear();

  /**
   * @brief Reserve space for vertices (optimization for large geometries)
   * @param count Number of vertices to reserve space for
   */
  void reserve(size_t count);

private:
  std::vector<msd_sim::Coordinate> vertices_;
};

}  // namespace msd_assets

#endif  // GEOMETRY_HPP
