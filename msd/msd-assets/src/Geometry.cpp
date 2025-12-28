#include "msd-assets/src/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace msd_assets
{

VisualGeometry::VisualGeometry(const std::vector<Eigen::Vector3d>& vertices)
  : cachedVertices_{computeVertexData(vertices)}
{
}


std::vector<Vertex> VisualGeometry::computeVertexData(
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

const std::vector<Vertex>& VisualGeometry::getVertices() const
{
  return cachedVertices_;
}

std::vector<Vertex> VisualGeometry::toGUIVertices(float r, float g, float b)
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

std::vector<uint8_t> VisualGeometry::serializeVertices() const
{
  const size_t blobSize = cachedVertices_.size() * sizeof(Vertex);
  std::vector<uint8_t> blob(blobSize);
  std::memcpy(blob.data(), cachedVertices_.data(), blobSize);
  return blob;
}

VisualGeometry VisualGeometry::deserializeVertices(
  const std::vector<uint8_t>& blob)
{
  if (blob.size() % sizeof(Vertex) != 0)
  {
    throw std::runtime_error(
      "Invalid BLOB size: not a multiple of Vertex size (36 bytes)");
  }

  VisualGeometry geom;
  const size_t vertexCount = blob.size() / sizeof(Vertex);
  const Vertex* vertexData = reinterpret_cast<const Vertex*>(blob.data());
  geom.cachedVertices_.assign(vertexData, vertexData + vertexCount);
  return geom;
}

size_t VisualGeometry::getVertexCount() const
{
  return cachedVertices_.size();
}

void VisualGeometry::populateMeshRecord(msd_transfer::MeshRecord& record) const
{
  // Populate vertex data
  const size_t vertexBlobSize = cachedVertices_.size() * sizeof(Vertex);
  record.vertex_data.resize(vertexBlobSize);
  std::memcpy(
    record.vertex_data.data(), cachedVertices_.data(), vertexBlobSize);
  record.vertex_count = static_cast<uint32_t>(cachedVertices_.size());
  record.triangle_count = record.vertex_count / 3;
}

VisualGeometry VisualGeometry::fromMeshRecord(
  const msd_transfer::MeshRecord& record)
{
  // Validate vertex_data blob size
  if (record.vertex_data.size() % sizeof(Vertex) != 0 ||
      record.vertex_data.size() == 0)
  {
    throw std::runtime_error("Invalid vertex_data BLOB size: not a multiple of "
                             "Vertex size (36 bytes)");
  }

  // Deserialize cached vertex data
  const size_t vertexCount = record.vertex_data.size() / sizeof(Vertex);
  const Vertex* vertexData =
    reinterpret_cast<const Vertex*>(record.vertex_data.data());

  // Create VisualGeometry with empty constructor, then populate
  VisualGeometry geom;
  geom.cachedVertices_.assign(vertexData, vertexData + vertexCount);

  return geom;
}

// CollisionGeometry implementation

CollisionGeometry::CollisionGeometry(
  const std::vector<Eigen::Vector3d>& vertices)
  : hullVertices_{vertices}
{
  calculateBoundingBox();
}

size_t CollisionGeometry::getVertexCount() const
{
  return hullVertices_.size();
}

const BoundingBox& CollisionGeometry::getBoundingBox() const
{
  return boundingBox_;
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

void CollisionGeometry::calculateBoundingBox()
{
  if (hullVertices_.empty())
  {
    boundingBox_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    return;
  }

  boundingBox_.min_x = boundingBox_.min_y = boundingBox_.min_z =
    std::numeric_limits<float>::max();
  boundingBox_.max_x = boundingBox_.max_y = boundingBox_.max_z =
    std::numeric_limits<float>::lowest();

  // Calculate AABB from hull vertices
  for (const auto& v : hullVertices_)
  {
    boundingBox_.min_x =
      std::min(boundingBox_.min_x, static_cast<float>(v.x()));
    boundingBox_.min_y =
      std::min(boundingBox_.min_y, static_cast<float>(v.y()));
    boundingBox_.min_z =
      std::min(boundingBox_.min_z, static_cast<float>(v.z()));
    boundingBox_.max_x =
      std::max(boundingBox_.max_x, static_cast<float>(v.x()));
    boundingBox_.max_y =
      std::max(boundingBox_.max_y, static_cast<float>(v.y()));
    boundingBox_.max_z =
      std::max(boundingBox_.max_z, static_cast<float>(v.z()));
  }

  // Calculate bounding sphere radius (from AABB center)
  float center_x = (boundingBox_.min_x + boundingBox_.max_x) * 0.5f;
  float center_y = (boundingBox_.min_y + boundingBox_.max_y) * 0.5f;
  float center_z = (boundingBox_.min_z + boundingBox_.max_z) * 0.5f;

  float maxDistSq = 0.0f;
  for (const auto& v : hullVertices_)
  {
    float dx = static_cast<float>(v.x()) - center_x;
    float dy = static_cast<float>(v.y()) - center_y;
    float dz = static_cast<float>(v.z()) - center_z;
    float distSq = dx * dx + dy * dy + dz * dz;
    maxDistSq = std::max(maxDistSq, distSq);
  }

  boundingBox_.radius = std::sqrt(maxDistSq);
}

const std::vector<Eigen::Vector3d>& CollisionGeometry::getHullVertices() const
{
  return hullVertices_;
}

void CollisionGeometry::populateMeshRecord(
  msd_transfer::CollisionMeshRecord& record) const
{
  // Populate hull data
  record.hull_data = serializeHullVertices();
  record.hull_vertex_count = static_cast<uint32_t>(hullVertices_.size());

  // Populate bounding box metadata
  record.aabb_min_x = boundingBox_.min_x;
  record.aabb_min_y = boundingBox_.min_y;
  record.aabb_min_z = boundingBox_.min_z;
  record.aabb_max_x = boundingBox_.max_x;
  record.aabb_max_y = boundingBox_.max_y;
  record.aabb_max_z = boundingBox_.max_z;
  record.bounding_radius = boundingBox_.radius;
}

CollisionGeometry CollisionGeometry::fromMeshRecord(
  const msd_transfer::CollisionMeshRecord& record)
{
  // Deserialize hull vertices
  if (record.hull_data.empty())
  {
    throw std::runtime_error("CollisionMeshRecord has empty hull_data");
  }

  auto hullVertices = deserializeHullVertices(record.hull_data);

  // Create CollisionGeometry with hull vertices
  CollisionGeometry geom(hullVertices);

  // Restore bounding box from record (instead of recalculating)
  geom.boundingBox_.min_x = record.aabb_min_x;
  geom.boundingBox_.min_y = record.aabb_min_y;
  geom.boundingBox_.min_z = record.aabb_min_z;
  geom.boundingBox_.max_x = record.aabb_max_x;
  geom.boundingBox_.max_y = record.aabb_max_y;
  geom.boundingBox_.max_z = record.aabb_max_z;
  geom.boundingBox_.radius = record.bounding_radius;

  return geom;
}


}  // namespace msd_assets
