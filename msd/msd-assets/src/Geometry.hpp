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
 * @brief Compute vertex data with normals from source coordinates
 * @param vertices Source coordinate data
 */
std::vector<Vertex> computeVertexData(
  const std::vector<Eigen::Vector3d>& vertices);


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
template <typename T>
class BaseGeometry
{
public:
  /**
   * @brief Constructor with raw vertex data
   * @param rawVertices Vector of 3D coordinates (Eigen::Vector3d)
   * @param objectId Unique identifier for the geometry
   *
   * For VisualGeometry (T=Vertex): Computes normals and converts to Vertex
   * format. For CollisionGeometry (T=Eigen::Vector3d): Stores raw vertices
   * directly.
   */
  explicit BaseGeometry(const msd_transfer::MeshRecord& record,
                        uint32_t objectId = 0)
    : objectId_{objectId}
  {
    // Validate vertex_data blob size
    if (record.vertex_data.size() % sizeof(T) != 0 ||
        record.vertex_data.size() == 0)
    {
      throw std::runtime_error(
        "Invalid vertex_data BLOB size: not a multiple of "
        "Vertex size (36 bytes)");
    }

    // Deserialize cached vertex data using STL range constructor
    const size_t vertexCount = record.vertex_data.size() / sizeof(T);
    const Eigen::Vector3d* vertexBegin =
      reinterpret_cast<const Eigen::Vector3d*>(record.vertex_data.data());
    const Eigen::Vector3d* vertexEnd = vertexBegin + vertexCount;

    std::vector<Eigen::Vector3d> vertices(vertexBegin, vertexEnd);

    if constexpr (std::is_same_v<T, Vertex>)
    {
      // VisualGeometry: compute normals and convert to Vertex format
      cachedVertices_ = computeVertexData(vertices);
    }
    else
    {
      // CollisionGeometry: direct assignment of raw coordinates
      cachedVertices_ = std::move(vertices);
    }
  }

  /**
   * @brief Get number of vertices
   * @return Number of vertices in the cached vertex data
   */
  size_t getVertexCount() const
  {
    return cachedVertices_.size();
  }

  /**
   * @brief Serialize geometry to binary BLOB for database storage
   * @return Binary representation of Coordinate array (x, y, z positions)
   *
   * BLOB format: [x1,y1,z1][x2,y2,z2]...[xN,yN,zN]
   * Each Coordinate is 24 bytes: 3 doubles × 8 bytes
   * Colors and normals are computed on-demand via toGUIVertices()
   */
  std::vector<uint8_t> serializeVertices() const
  {
    const size_t blobSize = cachedVertices_.size() * sizeof(T);
    std::vector<uint8_t> blob(blobSize);
    std::memcpy(blob.data(), cachedVertices_.data(), blobSize);
    return blob;
  }


  /**
   * @brief Get cached GUI vertices (positions + normals, no color)
   * @return Const reference to cached vertex data
   *
   * Lazily computes and caches vertex data on first access.
   * Use toGUIVertices() if you need to apply colors.
   */
  const std::vector<T>& getVertices() const
  {
    return cachedVertices_;
  }

  /**
   * @brief Populate MeshRecord with geometry data
   * @param record MeshRecord to populate
   *
   * Populates vertex_data and bounding box metadata.
   * Note: Does not populate hull_data - use ConvexHull::serializeToBlob()
   * separately.
   */
  msd_transfer::MeshRecord populateMeshRecord() const
  {
    msd_transfer::MeshRecord record;
    // Populate vertex data
    const size_t vertexBlobSize = cachedVertices_.size() * sizeof(T);
    record.vertex_data.resize(vertexBlobSize);
    std::memcpy(
      record.vertex_data.data(), cachedVertices_.data(), vertexBlobSize);
    record.vertex_count = static_cast<uint32_t>(cachedVertices_.size());
    return record;
  }

private:
  uint32_t objectId_;

  std::vector<T> cachedVertices_;
};

using CollisionGeometry = BaseGeometry<Eigen::Vector3d>;
using VisualGeometry = BaseGeometry<Vertex>;


}  // namespace msd_assets

#endif  // GEOMETRY_HPP
