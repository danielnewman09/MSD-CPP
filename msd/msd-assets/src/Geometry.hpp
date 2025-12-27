#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <Eigen/Dense>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <vector>
#include "msd-transfer/src/MeshRecord.hpp"

namespace msd_assets
{

struct Vertex
{
  float position[3];  // Position (x, y, z)
  float color[3];     // Color (r, g, b)
  float normal[3];    // Normal vector (x, y, z)
};

/**
 * @brief Bounding volume metadata for geometry
 */
struct BoundingBox
{
  float min_x, min_y, min_z;
  float max_x, max_y, max_z;
  float radius;  // Bounding sphere radius
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
class BaseGeometry
{
public:
  /**
   * @brief Default constructor - creates empty geometry
   */
  BaseGeometry() = default;

  /**
   * @brief Constructor with vertex list
   * @param vertices Vector of 3D coordinates representing the geometry
   */
  explicit BaseGeometry(const std::vector<Eigen::Vector3d>& vertices);

  /**
   * @brief Convert Geometry to Vertex vector with colors
   * @param r Red component (0.0-1.0)
   * @param g Green component (0.0-1.0)
   * @param b Blue component (0.0-1.0)
   * @return Vector of Vertex structs ready for GPU upload
   *
   * Note: Vertex data (positions + normals) is cached after first computation.
   * Only colors are applied per-call.
   */
  std::vector<Vertex> toGUIVertices(float r = 1.0f,
                                    float g = 1.0f,
                                    float b = 1.0f);

  /**
   * @brief Get number of vertices
   * @return Number of vertices in the cached vertex data
   */
  size_t getVertexCount() const;

  /**
   * @brief Serialize geometry to binary BLOB for database storage
   * @return Binary representation of Coordinate array (x, y, z positions)
   *
   * BLOB format: [x1,y1,z1][x2,y2,z2]...[xN,yN,zN]
   * Each Coordinate is 24 bytes: 3 doubles × 8 bytes
   * Colors and normals are computed on-demand via toGUIVertices()
   */
  std::vector<uint8_t> serializeVertices() const;

  /**
   * @brief Deserialize geometry from binary BLOB
   * @param blob Binary data from database (Coordinate array)
   * @return Reconstructed BaseGeometry object
   * @throws std::runtime_error if blob size is not a multiple of 24 bytes
   */
  static BaseGeometry deserializeVertices(const std::vector<uint8_t>& blob);

  /**
   * @brief Get cached GUI vertices (positions + normals, no color)
   * @return Const reference to cached vertex data
   *
   * Lazily computes and caches vertex data on first access.
   * Use toGUIVertices() if you need to apply colors.
   */
  const std::vector<Vertex>& getVertices() const;

  /**
   * @brief Populate MeshRecord with geometry data
   * @param record MeshRecord to populate
   *
   * Populates vertex_data and bounding box metadata.
   * Note: Does not populate hull_data - use ConvexHull::serializeToBlob()
   * separately.
   */
  void populateMeshRecord(msd_transfer::MeshRecord& record) const;

  /**
   * @brief Create BaseGeometry from MeshRecord
   * @param record MeshRecord from database
   * @return BaseGeometry object with pre-cached vertex data
   */
  static BaseGeometry fromMeshRecord(const msd_transfer::MeshRecord& record);

protected:
  /**
   * @brief Compute vertex data with normals from source coordinates
   * @param vertices Source coordinate data
   */
  static std::vector<Vertex> computeVertexData(
    const std::vector<Eigen::Vector3d>& vertices);

  std::vector<Vertex> cachedVertices_;
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
class CollisionGeometry : public BaseGeometry
{
public:
  /**
   * @brief Default constructor - creates empty geometry
   */
  CollisionGeometry() = default;

  /**
   * @brief Constructor with vertex list
   * @param vertices Vector of 3D coordinates representing the geometry
   */
  explicit CollisionGeometry(const std::vector<Eigen::Vector3d>& vertices);

  /**
   * @brief Get number of vertices
   * @return Number of vertices in the cached vertex data
   */
  size_t getVertexCount() const;

  /**
   * @brief Serialize geometry to binary BLOB for database storage
   * @return Binary representation of Coordinate array (x, y, z positions)
   *
   * BLOB format: [x1,y1,z1][x2,y2,z2]...[xN,yN,zN]
   * Each Coordinate is 24 bytes: 3 doubles × 8 bytes
   * Colors and normals are computed on-demand via toGUIVertices()
   */
  std::vector<uint8_t> serializeHullVertices() const;

  /**
   * @brief Deserialize geometry from binary BLOB
   * @param blob Binary data from database (Coordinate array)
   * @return Reconstructed Geometry object
   * @throws std::runtime_error if blob size is not a multiple of 24 bytes
   */
  static std::vector<Eigen::Vector3d> deserializeHullVertices(
    const std::vector<uint8_t>& blob);

  /**
   * @brief Calculate axis-aligned bounding box and bounding sphere
   * @return BoundingBox containing AABB min/max and sphere radius
   */
  BoundingBox calculateBoundingBox() const;

  /**
   * @brief Get source vertices (coordinates)
   * @return Vector of coordinate data
   */
  const std::vector<Eigen::Vector3d>& getHullVertices() const;

  /**
   * @brief Populate MeshRecord with geometry data
   * @param record MeshRecord to populate
   *
   * Populates vertex_data and bounding box metadata.
   * Note: Does not populate hull_data - use ConvexHull::serializeToBlob()
   * separately.
   */
  void populateMeshRecord(msd_transfer::CollisionMeshRecord& record) const;

  /**
   * @brief Create CollisionGeometry from CollisionMeshRecord
   * @param record CollisionMeshRecord from database
   * @return CollisionGeometry object with hull vertices
   */
  static CollisionGeometry fromMeshRecord(
    const msd_transfer::CollisionMeshRecord& record);

private:
  std::vector<Eigen::Vector3d> hullVertices_;
};

}  // namespace msd_assets

#endif  // GEOMETRY_HPP
