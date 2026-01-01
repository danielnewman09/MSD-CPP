#include "msd-assets/src/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace msd_assets
{


std::vector<Vertex> computeVertexData(
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

// std::vector<Vertex> VisualGeometry::toGUIVertices(float r, float g, float b)
// {
//   // Copy cached vertices and apply colors
//   std::vector<Vertex> vertices = cachedVertices_;
//   for (auto& v : vertices)
//   {
//     v.color[0] = r;
//     v.color[1] = g;
//     v.color[2] = b;
//   }

//   return vertices;
// }


// CollisionGeometry implementation


// void CollisionGeometry::calculateBoundingBox()
// {
//   if (hullVertices_.empty())
//   {
//     boundingBox_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//     return;
//   }

//   boundingBox_.min_x = boundingBox_.min_y = boundingBox_.min_z =
//     std::numeric_limits<float>::max();
//   boundingBox_.max_x = boundingBox_.max_y = boundingBox_.max_z =
//     std::numeric_limits<float>::lowest();

//   // Calculate AABB from hull vertices
//   for (const auto& v : hullVertices_)
//   {
//     boundingBox_.min_x =
//       std::min(boundingBox_.min_x, static_cast<float>(v.x()));
//     boundingBox_.min_y =
//       std::min(boundingBox_.min_y, static_cast<float>(v.y()));
//     boundingBox_.min_z =
//       std::min(boundingBox_.min_z, static_cast<float>(v.z()));
//     boundingBox_.max_x =
//       std::max(boundingBox_.max_x, static_cast<float>(v.x()));
//     boundingBox_.max_y =
//       std::max(boundingBox_.max_y, static_cast<float>(v.y()));
//     boundingBox_.max_z =
//       std::max(boundingBox_.max_z, static_cast<float>(v.z()));
//   }

//   // Calculate bounding sphere radius (from AABB center)
//   float center_x = (boundingBox_.min_x + boundingBox_.max_x) * 0.5f;
//   float center_y = (boundingBox_.min_y + boundingBox_.max_y) * 0.5f;
//   float center_z = (boundingBox_.min_z + boundingBox_.max_z) * 0.5f;

//   float maxDistSq = 0.0f;
//   for (const auto& v : hullVertices_)
//   {
//     float dx = static_cast<float>(v.x()) - center_x;
//     float dy = static_cast<float>(v.y()) - center_y;
//     float dz = static_cast<float>(v.z()) - center_z;
//     float distSq = dx * dx + dy * dy + dz * dz;
//     maxDistSq = std::max(maxDistSq, distSq);
//   }

//   boundingBox_.radius = std::sqrt(maxDistSq);
// }

// void CollisionGeometry::populateMeshRecord(
//   msd_transfer::CollisionMeshRecord& record) const
// {
//   // Populate hull data
//   record.hull_data = serializeHullVertices();
//   record.hull_vertex_count = static_cast<uint32_t>(hullVertices_.size());

//   // Populate bounding box metadata
//   record.aabb_min_x = boundingBox_.min_x;
//   record.aabb_min_y = boundingBox_.min_y;
//   record.aabb_min_z = boundingBox_.min_z;
//   record.aabb_max_x = boundingBox_.max_x;
//   record.aabb_max_y = boundingBox_.max_y;
//   record.aabb_max_z = boundingBox_.max_z;
//   record.bounding_radius = boundingBox_.radius;
// }


}  // namespace msd_assets
