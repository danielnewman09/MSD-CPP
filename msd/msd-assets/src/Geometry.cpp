#include "msd-assets/src/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace msd_assets
{

BaseGeometry::BaseGeometry(const std::vector<Eigen::Vector3d>& vertices)
  : cachedVertices_{computeVertexData(vertices)}
{
}


std::vector<Vertex> BaseGeometry::computeVertexData(
  const std::vector<Eigen::Vector3d>& vertices)
{
  std::vector<Vertex> result;
  result.reserve(vertices.size());

  // Process triangles (every 3 vertices form a triangle)
  for (size_t i = 0; i + 2 < vertices.size(); i += 3)
  {
    const auto& v0 = vertices[i];
    const auto& v1 = vertices[i + 1];
    const auto& v2 = vertices[i + 2];

    // Calculate two edge vectors using Eigen operations
    auto edge1 = v1 - v0;
    auto edge2 = v2 - v0;

    // Calculate normal using cross product and normalize
    auto normal = edge1.cross(edge2).normalized();

    // Convert double precision coordinates to float for GPU
    // Add all three vertices of the triangle with the same normal
    // Use default white color (1,1,1) for cached version
    result.push_back({{static_cast<float>(v0.x()),
                       static_cast<float>(v0.y()),
                       static_cast<float>(v0.z())},
                      {1.0f, 1.0f, 1.0f},
                      {static_cast<float>(normal.x()),
                       static_cast<float>(normal.y()),
                       static_cast<float>(normal.z())}});
    result.push_back({{static_cast<float>(v1.x()),
                       static_cast<float>(v1.y()),
                       static_cast<float>(v1.z())},
                      {1.0f, 1.0f, 1.0f},
                      {static_cast<float>(normal.x()),
                       static_cast<float>(normal.y()),
                       static_cast<float>(normal.z())}});
    result.push_back({{static_cast<float>(v2.x()),
                       static_cast<float>(v2.y()),
                       static_cast<float>(v2.z())},
                      {1.0f, 1.0f, 1.0f},
                      {static_cast<float>(normal.x()),
                       static_cast<float>(normal.y()),
                       static_cast<float>(normal.z())}});
  }

  return result;
}

const std::vector<Vertex>& BaseGeometry::getVertices() const
{
  return cachedVertices_;
}

std::vector<Vertex> BaseGeometry::toGUIVertices(float r, float g, float b)
{
  // Copy cached vertices and apply colors
  std::vector<Vertex> vertices = cachedVertices_;
  for (auto& v : vertices)
  {
    v.color[0] = r;
    v.color[1] = g;
    v.color[2] = b;
  }

  return vertices;
}

std::vector<uint8_t> BaseGeometry::serializeVertices() const
{
  const size_t blobSize = cachedVertices_.size() * sizeof(Vertex);
  std::vector<uint8_t> blob(blobSize);
  std::memcpy(blob.data(), cachedVertices_.data(), blobSize);
  return blob;
}

BaseGeometry BaseGeometry::deserializeVertices(const std::vector<uint8_t>& blob)
{
  if (blob.size() % sizeof(Vertex) != 0)
  {
    throw std::runtime_error(
      "Invalid BLOB size: not a multiple of Vertex size (36 bytes)");
  }

  BaseGeometry geom;
  const size_t vertexCount = blob.size() / sizeof(Vertex);
  const Vertex* vertexData = reinterpret_cast<const Vertex*>(blob.data());
  geom.cachedVertices_.assign(vertexData, vertexData + vertexCount);
  return geom;
}

size_t BaseGeometry::getVertexCount() const
{
  return cachedVertices_.size();
}

void BaseGeometry::populateMeshRecord(msd_transfer::MeshRecord& record) const
{
  // Populate vertex data
  const size_t vertexBlobSize = cachedVertices_.size() * sizeof(Vertex);
  record.vertex_data.resize(vertexBlobSize);
  std::memcpy(
    record.vertex_data.data(), cachedVertices_.data(), vertexBlobSize);
  record.vertex_count = static_cast<uint32_t>(cachedVertices_.size());
  record.triangle_count = record.vertex_count / 3;
}

BaseGeometry BaseGeometry::fromMeshRecord(
  const msd_transfer::MeshRecord& record)
{
  // Validate vertex_data blob size
  if (record.vertex_data.size() % sizeof(Vertex) != 0)
  {
    throw std::runtime_error("Invalid vertex_data BLOB size: not a multiple of "
                             "Vertex size (36 bytes)");
  }

  // Deserialize cached vertex data
  const size_t vertexCount = record.vertex_data.size() / sizeof(Vertex);
  const Vertex* vertexData =
    reinterpret_cast<const Vertex*>(record.vertex_data.data());

  // Create BaseGeometry with empty constructor, then populate
  BaseGeometry geom;
  geom.cachedVertices_.assign(vertexData, vertexData + vertexCount);

  return geom;
}

// CollisionGeometry implementation

CollisionGeometry::CollisionGeometry(
  const std::vector<Eigen::Vector3d>& vertices)
  : BaseGeometry(vertices), hullVertices_{vertices}
{
}

size_t CollisionGeometry::getVertexCount() const
{
  return hullVertices_.size();
}

std::vector<uint8_t> CollisionGeometry::serializeHullVertices() const
{
  const size_t blobSize = hullVertices_.size() * 3 * sizeof(double);
  std::vector<uint8_t> blob(blobSize);

  double* dest = reinterpret_cast<double*>(blob.data());
  for (const auto& vertex : hullVertices_)
  {
    *dest++ = vertex.x();
    *dest++ = vertex.y();
    *dest++ = vertex.z();
  }

  return blob;
}

std::vector<Eigen::Vector3d> CollisionGeometry::deserializeHullVertices(
  const std::vector<uint8_t>& blob)
{
  const size_t coordSize = 3 * sizeof(double);
  if (blob.size() % coordSize != 0)
  {
    throw std::runtime_error(
      "Invalid hull BLOB size: not a multiple of coordinate size (24 bytes)");
  }

  const size_t vertexCount = blob.size() / coordSize;
  const double* src = reinterpret_cast<const double*>(blob.data());

  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(vertexCount);

  for (size_t i = 0; i < vertexCount; ++i)
  {
    double x = *src++;
    double y = *src++;
    double z = *src++;
    vertices.emplace_back(x, y, z);
  }

  return vertices;
}

BoundingBox CollisionGeometry::calculateBoundingBox() const
{
  if (cachedVertices_.empty())
  {
    return {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  }

  BoundingBox bbox;
  bbox.min_x = bbox.min_y = bbox.min_z = std::numeric_limits<float>::max();
  bbox.max_x = bbox.max_y = bbox.max_z = std::numeric_limits<float>::lowest();

  // Calculate AABB from vertex positions
  for (const auto& v : cachedVertices_)
  {
    bbox.min_x = std::min(bbox.min_x, v.position[0]);
    bbox.min_y = std::min(bbox.min_y, v.position[1]);
    bbox.min_z = std::min(bbox.min_z, v.position[2]);
    bbox.max_x = std::max(bbox.max_x, v.position[0]);
    bbox.max_y = std::max(bbox.max_y, v.position[1]);
    bbox.max_z = std::max(bbox.max_z, v.position[2]);
  }

  // Calculate bounding sphere radius (from AABB center)
  float center_x = (bbox.min_x + bbox.max_x) * 0.5f;
  float center_y = (bbox.min_y + bbox.max_y) * 0.5f;
  float center_z = (bbox.min_z + bbox.max_z) * 0.5f;

  float maxDistSq = 0.0f;
  for (const auto& v : cachedVertices_)
  {
    float dx = v.position[0] - center_x;
    float dy = v.position[1] - center_y;
    float dz = v.position[2] - center_z;
    float distSq = dx * dx + dy * dy + dz * dz;
    maxDistSq = std::max(maxDistSq, distSq);
  }

  bbox.radius = std::sqrt(maxDistSq);

  return bbox;
}

const std::vector<Eigen::Vector3d>& CollisionGeometry::getHullVertices() const
{
  return hullVertices_;
}

void CollisionGeometry::populateMeshRecord(
  msd_transfer::CollisionMeshRecord& record) const
{
  // First populate the base visual mesh data
  BaseGeometry::populateMeshRecord(record);

  // Now populate hull data
  record.hull_data = serializeHullVertices();
  record.hull_vertex_count = static_cast<uint32_t>(hullVertices_.size());

  // Populate bounding box metadata
  BoundingBox bbox = calculateBoundingBox();
  record.aabb_min_x = bbox.min_x;
  record.aabb_min_y = bbox.min_y;
  record.aabb_min_z = bbox.min_z;
  record.aabb_max_x = bbox.max_x;
  record.aabb_max_y = bbox.max_y;
  record.aabb_max_z = bbox.max_z;
  record.bounding_radius = bbox.radius;
}

CollisionGeometry CollisionGeometry::fromMeshRecord(
  const msd_transfer::CollisionMeshRecord& record)
{
  // Validate vertex_data blob size
  if (record.vertex_data.size() % sizeof(Vertex) != 0)
  {
    throw std::runtime_error("Invalid vertex_data BLOB size: not a multiple of "
                             "Vertex size (36 bytes)");
  }

  // Deserialize cached vertex data
  const size_t vertexCount = record.vertex_data.size() / sizeof(Vertex);
  const Vertex* vertexData =
    reinterpret_cast<const Vertex*>(record.vertex_data.data());

  // Extract visual coordinates from vertex data
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(vertexCount);
  for (size_t i = 0; i < vertexCount; ++i)
  {
    vertices.emplace_back(static_cast<double>(vertexData[i].position[0]),
                          static_cast<double>(vertexData[i].position[1]),
                          static_cast<double>(vertexData[i].position[2]));
  }

  // Create CollisionGeometry with visual vertices
  CollisionGeometry geom(vertices);

  // Deserialize hull vertices if present
  if (!record.hull_data.empty())
  {
    geom.hullVertices_ = deserializeHullVertices(record.hull_data);
  }

  return geom;
}


}  // namespace msd_assets
