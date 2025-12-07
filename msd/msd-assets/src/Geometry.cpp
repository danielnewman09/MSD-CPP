#include "msd-assets/src/Geometry.hpp"

namespace msd_assets
{

Geometry::Geometry(const std::vector<msd_sim::Coordinate>& vertices)
  : vertices_{vertices}
{
}

void Geometry::setVertices(const std::vector<msd_sim::Coordinate>& vertices)
{
  vertices_ = vertices;
}

void Geometry::addVertex(const msd_sim::Coordinate& vertex)
{
  vertices_.push_back(vertex);
}

const std::vector<msd_sim::Coordinate>& Geometry::getVertices() const
{
  return vertices_;
}

size_t Geometry::getVertexCount() const
{
  return vertices_.size();
}

void Geometry::clear()
{
  vertices_.clear();
}

void Geometry::reserve(size_t count)
{
  vertices_.reserve(count);
}

}  // namespace msd_assets
