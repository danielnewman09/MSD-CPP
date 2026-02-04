#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

extern "C"
{
#include <libqhull_r/geom_r.h>
#include <libqhull_r/libqhull_r.h>
}

#include "msd-sim/src/Physics/Collision/GJK.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace msd_sim
{

ConvexHull::ConvexHull()
  : volume_{std::numeric_limits<double>::quiet_NaN()},
    surfaceArea_{std::numeric_limits<double>::quiet_NaN()},
    boundingBoxMin_{0.0, 0.0, 0.0},
    boundingBoxMax_{0.0, 0.0, 0.0},
    centroid_{0.0, 0.0, 0.0}
{
}

ConvexHull::ConvexHull(const msd_assets::CollisionGeometry& geometry)
  : volume_{std::numeric_limits<double>::quiet_NaN()},
    surfaceArea_{std::numeric_limits<double>::quiet_NaN()},
    boundingBoxMin_{0.0, 0.0, 0.0},
    boundingBoxMax_{0.0, 0.0, 0.0},
    centroid_{0.0, 0.0, 0.0}
{
  computeHull(geometry.getVertices());
}


const std::vector<Coordinate>& ConvexHull::getVertices() const
{
  return vertices_;
}

const std::vector<Facet>& ConvexHull::getFacets() const
{
  return facets_;
}

const Facet& ConvexHull::getFacetAlignedWith(
  const Eigen::Vector3d& normal) const
{
  auto it =
    std::max_element(facets_.begin(),
                     facets_.end(),
                     [&normal](const Facet& a, const Facet& b)
                     { return a.normal.dot(normal) < b.normal.dot(normal); });
  return *it;
}

std::vector<std::reference_wrapper<const Facet>>
ConvexHull::getFacetsAlignedWith(const Eigen::Vector3d& normal,
                                 double tolerance) const
{
  // First pass: find maximum alignment
  double maxDot =
    std::max_element(facets_.begin(),
                     facets_.end(),
                     [&normal](const Facet& a, const Facet& b)
                     { return a.normal.dot(normal) < b.normal.dot(normal); })
      ->normal.dot(normal);

  // Second pass: collect all facets within tolerance of the maximum
  std::vector<std::reference_wrapper<const Facet>> result;
  for (const auto& facet : facets_)
  {
    if (facet.normal.dot(normal) >= maxDot - tolerance)
    {
      result.push_back(std::cref(facet));
    }
  }

  return result;
}

size_t ConvexHull::getVertexCount() const
{
  return vertices_.size();
}

size_t ConvexHull::getFacetCount() const
{
  return facets_.size();
}

double ConvexHull::getVolume() const
{
  return volume_;
}

double ConvexHull::getSurfaceArea() const
{
  return surfaceArea_;
}

Coordinate ConvexHull::getCentroid() const
{
  return centroid_;
}

bool ConvexHull::contains(const Coordinate& point, double epsilon) const
{
  // A point is inside the convex hull if it's on the negative side
  // (or within epsilon) of all half-space planes defined by the facets
  for (const auto& facet : facets_)
  {
    const Coordinate& v0 = vertices_[facet.vertexIndices[0]];

    // Distance from point to plane: d = normal · (point - v0)
    double distance = facet.normal.dot(point - v0);

    // If point is on the positive side (outside), it's not contained
    if (distance > epsilon)
    {
      return false;
    }
  }

  return true;
}

double ConvexHull::signedDistance(const Coordinate& point) const
{
  if (facets_.empty())
  {
    return std::numeric_limits<double>::infinity();
  }

  // Find the maximum signed distance to all facet planes
  // Negative means inside, positive means outside
  double maxDistance = -std::numeric_limits<double>::infinity();

  for (const auto& facet : facets_)
  {
    const Coordinate& v0 = vertices_[facet.vertexIndices[0]];
    double distance = facet.normal.dot(point - v0);
    maxDistance = std::max(maxDistance, distance);
  }

  return maxDistance;
}

ConvexHull::BoundingBox ConvexHull::getBoundingBox() const
{
  // Bounding box is computed directly by Qhull during computeHull()
  // No need to recalculate - Qhull provides accurate bounding box
  return BoundingBox{boundingBoxMin_, boundingBoxMax_};
}

bool ConvexHull::isValid() const
{
  return !vertices_.empty() && !facets_.empty();
}

// computeHull is now a template defined in the header file

void ConvexHull::extractHullData(qhT* qh)
{
  vertices_.clear();
  facets_.clear();

  // Create a map from vertex ID to our vertex index
  std::unordered_map<int, size_t> vertexIdMap;

  // Extract vertices
  vertexT* vertex;
  size_t vertexIndex = 0;

  FORALLvertices
  {
    pointT* point = vertex->point;
    vertices_.emplace_back(point[0], point[1], point[2]);
    vertexIdMap[qh_pointid(qh, point)] = vertexIndex++;
  }

  // Extract facets (triangulated by Qt option)
  facetT* facet;

  FORALLfacets
  {
    if (!facet->upperdelaunay && facet->simplicial)
    {
      // Count vertices in this facet
      vertexT** vertexp;
      int vertexCount = 0;

      FOREACHvertex_(facet->vertices)
      {
        vertexCount++;
      }

      // Qt option should triangulate, but check anyway
      if (vertexCount != 3)
      {
        continue;
      }

      Facet hullFacet;
      size_t idx = 0;

      FOREACHvertex_(facet->vertices)
      {
        int pointId = qh_pointid(qh, vertex->point);
        hullFacet.vertexIndices[idx++] = vertexIdMap[pointId];
      }

      // Get facet normal (outward-facing)
      hullFacet.normal =
        Coordinate{facet->normal[0], facet->normal[1], facet->normal[2]};

      // Normalize the normal vector
      double normalLength = hullFacet.normal.norm();
      if (normalLength > 1e-8)
      {
        hullFacet.normal /= normalLength;
      }

      // Get offset (distance from origin to plane)
      hullFacet.offset = facet->offset;

      facets_.push_back(hullFacet);
    }
  }
}


void ConvexHull::computeCentroid()
{
  if (vertices_.empty())
  {
    centroid_ = Coordinate{0.0, 0.0, 0.0};
  }

  // Compute volume-weighted centroid using tetrahedron decomposition
  Coordinate centroidSum{0.0, 0.0, 0.0};
  double totalVolume = 0.0;

  for (const auto& facet : facets_)
  {
    const Coordinate& v0 = vertices_[facet.vertexIndices[0]];
    const Coordinate& v1 = vertices_[facet.vertexIndices[1]];
    const Coordinate& v2 = vertices_[facet.vertexIndices[2]];

    // Volume of tetrahedron (with origin as 4th vertex)
    double tetVol = std::abs(v0.dot(v1.cross(v2))) / 6.0;

    // Centroid of tetrahedron is average of 4 vertices (origin + 3 triangle
    // vertices)
    Coordinate tetCentroid = (v0 + v1 + v2) / 4.0;

    centroidSum += tetVol * tetCentroid;
    totalVolume += tetVol;
  }

  if (totalVolume > 0.0)
  {
    centroid_ = centroidSum / totalVolume;
  }
  else
  {
    centroid_ = Coordinate{0.0, 0.0, 0.0};
  }
}

}  // namespace msd_sim
