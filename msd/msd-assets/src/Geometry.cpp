#include "msd-assets/src/Geometry.hpp"

namespace msd_assets
{

Geometry::Geometry(const std::vector<msd_sim::Coordinate>& vertices)
  : vertices_{vertices}
{
}


std::vector<Vertex> Geometry::toGUIVertices(float r, float g, float b)
{
  std::vector<Vertex> vertices;
  vertices.reserve(vertices_.size());

  // Process triangles (every 3 vertices form a triangle)
  for (size_t i = 0; i + 2 < vertices_.size(); i += 3)
  {
    const auto& v0 = vertices_[i];
    const auto& v1 = vertices_[i + 1];
    const auto& v2 = vertices_[i + 2];

    // Calculate two edge vectors using Eigen operations
    auto edge1 = v1 - v0;
    auto edge2 = v2 - v0;

    // Calculate normal using cross product and normalize
    auto normal = edge1.cross(edge2).normalized();

    // Convert double precision coordinates to float for GPU
    // Add all three vertices of the triangle with the same normal
    vertices.push_back({{static_cast<float>(v0.x()),
                         static_cast<float>(v0.y()),
                         static_cast<float>(v0.z())},
                        {r, g, b},
                        {static_cast<float>(normal.x()),
                         static_cast<float>(normal.y()),
                         static_cast<float>(normal.z())}});
    vertices.push_back({{static_cast<float>(v1.x()),
                         static_cast<float>(v1.y()),
                         static_cast<float>(v1.z())},
                        {r, g, b},
                        {static_cast<float>(normal.x()),
                         static_cast<float>(normal.y()),
                         static_cast<float>(normal.z())}});
    vertices.push_back({{static_cast<float>(v2.x()),
                         static_cast<float>(v2.y()),
                         static_cast<float>(v2.z())},
                        {r, g, b},
                        {static_cast<float>(normal.x()),
                         static_cast<float>(normal.y()),
                         static_cast<float>(normal.z())}});
  }

  return vertices;
}

const std::vector<msd_sim::Coordinate>& Geometry::getVertices() const
{
  return vertices_;
}

size_t Geometry::getVertexCount() const
{
  return vertices_.size();
}


}  // namespace msd_assets
