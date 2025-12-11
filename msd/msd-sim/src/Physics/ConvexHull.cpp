#include "ConvexHull.hpp"
#include "msd-assets/src/Geometry.hpp"
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/QhullPoint.h>
#include <stdexcept>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>

namespace msd_sim {

ConvexHull::ConvexHull()
  : cachedVolume_(-1.0f)
  , cachedSurfaceArea_(-1.0f)
  , cachedCentroid_(0.0f, 0.0f, 0.0f)
  , centroidValid_(false)
  , inputPointCount_(0)
{
}

ConvexHull::ConvexHull(const std::vector<Coordinate>& points)
  : ConvexHull()
{
  if (points.empty()) {
    throw std::runtime_error("Cannot create convex hull from empty point set");
  }
  computeHull(points);
}

ConvexHull::ConvexHull(const ConvexHull& other)
  : vertices_(other.vertices_)
  , facets_(other.facets_)
  , cachedVolume_(other.cachedVolume_)
  , cachedSurfaceArea_(other.cachedSurfaceArea_)
  , cachedCentroid_(other.cachedCentroid_)
  , centroidValid_(other.centroidValid_)
  , inputPointCount_(other.inputPointCount_)
{
}

ConvexHull::ConvexHull(ConvexHull&& other) noexcept
  : vertices_(std::move(other.vertices_))
  , facets_(std::move(other.facets_))
  , cachedVolume_(other.cachedVolume_)
  , cachedSurfaceArea_(other.cachedSurfaceArea_)
  , cachedCentroid_(other.cachedCentroid_)
  , centroidValid_(other.centroidValid_)
  , inputPointCount_(other.inputPointCount_)
{
  other.invalidateCache();
}

ConvexHull& ConvexHull::operator=(const ConvexHull& other)
{
  if (this != &other) {
    vertices_ = other.vertices_;
    facets_ = other.facets_;
    cachedVolume_ = other.cachedVolume_;
    cachedSurfaceArea_ = other.cachedSurfaceArea_;
    cachedCentroid_ = other.cachedCentroid_;
    centroidValid_ = other.centroidValid_;
    inputPointCount_ = other.inputPointCount_;
  }
  return *this;
}

ConvexHull& ConvexHull::operator=(ConvexHull&& other) noexcept
{
  if (this != &other) {
    vertices_ = std::move(other.vertices_);
    facets_ = std::move(other.facets_);
    cachedVolume_ = other.cachedVolume_;
    cachedSurfaceArea_ = other.cachedSurfaceArea_;
    cachedCentroid_ = other.cachedCentroid_;
    centroidValid_ = other.centroidValid_;
    inputPointCount_ = other.inputPointCount_;
    other.invalidateCache();
  }
  return *this;
}

ConvexHull::~ConvexHull() = default;

ConvexHull ConvexHull::fromGeometry(const msd_assets::Geometry& geometry)
{
  const auto& vertices = geometry.getVertices();
  if (vertices.empty()) {
    throw std::runtime_error("Cannot create convex hull from empty geometry");
  }
  return ConvexHull(vertices);
}

ConvexHull ConvexHull::fromPoints(const std::vector<Coordinate>& points)
{
  return ConvexHull(points);
}

const std::vector<Coordinate>& ConvexHull::getVertices() const
{
  return vertices_;
}

const std::vector<ConvexHull::Facet>& ConvexHull::getFacets() const
{
  return facets_;
}

size_t ConvexHull::getVertexCount() const
{
  return vertices_.size();
}

size_t ConvexHull::getFacetCount() const
{
  return facets_.size();
}

float ConvexHull::volume() const
{
  // Volume is computed directly by Qhull and cached during computeHull()
  // No need to recalculate - Qhull provides accurate volume
  return cachedVolume_;
}

float ConvexHull::surfaceArea() const
{
  // Surface area is computed directly by Qhull and cached during computeHull()
  // No need to recalculate - Qhull provides accurate surface area
  return cachedSurfaceArea_;
}

Coordinate ConvexHull::centroid() const
{
  if (centroidValid_) {
    return cachedCentroid_;
  }

  if (vertices_.empty()) {
    cachedCentroid_ = Coordinate(0.0f, 0.0f, 0.0f);
    centroidValid_ = true;
    return cachedCentroid_;
  }

  // Compute volume-weighted centroid using tetrahedron decomposition
  Coordinate centroid(0.0f, 0.0f, 0.0f);
  float totalVolume = 0.0f;
  Coordinate origin(0.0f, 0.0f, 0.0f);

  for (const auto& facet : facets_) {
    const Coordinate& v0 = vertices_[facet.vertexIndices[0]];
    const Coordinate& v1 = vertices_[facet.vertexIndices[1]];
    const Coordinate& v2 = vertices_[facet.vertexIndices[2]];

    // Volume of tetrahedron
    float tetVol = std::abs(v0.dot(v1.cross(v2))) / 6.0f;

    // Centroid of tetrahedron is average of 4 vertices (origin + 3 triangle vertices)
    Coordinate tetCentroid = (origin + v0 + v1 + v2) / 4.0f;

    centroid += tetVol * tetCentroid;
    totalVolume += tetVol;
  }

  if (totalVolume > 0.0f) {
    cachedCentroid_ = centroid / totalVolume;
  } else {
    cachedCentroid_ = Coordinate(0.0f, 0.0f, 0.0f);
  }

  centroidValid_ = true;
  return cachedCentroid_;
}

bool ConvexHull::contains(const Coordinate& point, float epsilon) const
{
  // A point is inside the convex hull if it's on the negative side
  // (or within epsilon) of all half-space planes defined by the facets
  for (const auto& facet : facets_) {
    const Coordinate& v0 = vertices_[facet.vertexIndices[0]];

    // Distance from point to plane: d = normal · (point - v0)
    float distance = facet.normal.dot(point - v0);

    // If point is on the positive side (outside), it's not contained
    if (distance > epsilon) {
      return false;
    }
  }

  return true;
}

float ConvexHull::signedDistance(const Coordinate& point) const
{
  if (facets_.empty()) {
    return std::numeric_limits<float>::infinity();
  }

  // Find the maximum signed distance to all facet planes
  // Negative means inside, positive means outside
  float maxDistance = -std::numeric_limits<float>::infinity();

  for (const auto& facet : facets_) {
    const Coordinate& v0 = vertices_[facet.vertexIndices[0]];
    float distance = facet.normal.dot(point - v0);
    maxDistance = std::max(maxDistance, distance);
  }

  return maxDistance;
}

void ConvexHull::getBoundingBox(Coordinate& min, Coordinate& max) const
{
  if (vertices_.empty()) {
    min = Coordinate(0.0f, 0.0f, 0.0f);
    max = Coordinate(0.0f, 0.0f, 0.0f);
    return;
  }

  min = vertices_[0];
  max = vertices_[0];

  for (size_t i = 1; i < vertices_.size(); ++i) {
    min.x() = std::min(min.x(), vertices_[i].x());
    min.y() = std::min(min.y(), vertices_[i].y());
    min.z() = std::min(min.z(), vertices_[i].z());

    max.x() = std::max(max.x(), vertices_[i].x());
    max.y() = std::max(max.y(), vertices_[i].y());
    max.z() = std::max(max.z(), vertices_[i].z());
  }
}

bool ConvexHull::isValid() const
{
  return !vertices_.empty() && !facets_.empty() && volume() > 0.0f;
}

size_t ConvexHull::getInputPointCount() const
{
  return inputPointCount_;
}

void ConvexHull::computeHull(const std::vector<Coordinate>& points)
{
  inputPointCount_ = points.size();

  if (points.size() < 4) {
    throw std::runtime_error(
      "Cannot create 3D convex hull from fewer than 4 points");
  }

  try {
    // Convert points to Qhull format (flat array of doubles)
    std::vector<double> qhullPoints;
    qhullPoints.reserve(points.size() * 3);

    for (const auto& point : points) {
      qhullPoints.push_back(static_cast<double>(point.x()));
      qhullPoints.push_back(static_cast<double>(point.y()));
      qhullPoints.push_back(static_cast<double>(point.z()));
    }

    // Create Qhull object and compute convex hull
    // "Qt" = triangulated output (ensures all facets are triangles)
    orgQhull::Qhull qhull;
    qhull.runQhull("", 3, static_cast<int>(points.size()),
                   qhullPoints.data(), "Qt");

    if (qhull.qhullStatus() != 0) {
      std::ostringstream oss;
      oss << "Qhull failed with status " << qhull.qhullStatus();
      throw std::runtime_error(oss.str());
    }

    // Get volume and area directly from Qhull (more accurate than manual calculation)
    cachedVolume_ = static_cast<float>(qhull.volume());
    cachedSurfaceArea_ = static_cast<float>(qhull.area());

    // Extract hull data
    extractHullData(qhull);

  } catch (const std::exception& e) {
    std::ostringstream oss;
    oss << "Failed to compute convex hull: " << e.what();
    throw std::runtime_error(oss.str());
  }

  // Don't invalidate volume/area cache - already set from Qhull
  centroidValid_ = false;
}

void ConvexHull::extractHullData(const orgQhull::Qhull& qhull)
{
  vertices_.clear();
  facets_.clear();

  // Extract vertices
  auto vertexList = qhull.vertexList();
  std::vector<int> vertexIdMap(qhull.vertexCount(), -1);

  int vertexIndex = 0;
  for (const auto& vertex : vertexList) {
    auto point = vertex.point();
    vertices_.emplace_back(
      static_cast<float>(point[0]),
      static_cast<float>(point[1]),
      static_cast<float>(point[2])
    );
    vertexIdMap[vertex.id()] = vertexIndex++;
  }

  // Extract facets (triangulated)
  auto facetList = qhull.facetList();
  for (const auto& facet : facetList) {
    if (facet.isGood()) {
      auto vertices = facet.vertices();

      if (vertices.count() != 3) {
        // Qhull should triangulate with Qt option, but check anyway
        continue;
      }

      Facet hullFacet;
      int idx = 0;
      for (const auto& vertex : vertices) {
        hullFacet.vertexIndices[idx++] = vertexIdMap[vertex.id()];
      }

      // Get facet normal (outward-facing)
      auto normalCoords = facet.hyperplane().coordinates();
      hullFacet.normal = Coordinate(
        static_cast<float>(normalCoords[0]),
        static_cast<float>(normalCoords[1]),
        static_cast<float>(normalCoords[2])
      );

      // Normalize the normal vector
      float normalLength = hullFacet.normal.norm();
      if (normalLength > 1e-8f) {
        hullFacet.normal /= normalLength;
      }

      // Get offset (distance from origin to plane)
      hullFacet.offset = static_cast<float>(facet.hyperplane().offset());

      facets_.push_back(hullFacet);
    }
  }
}

void ConvexHull::invalidateCache()
{
  cachedVolume_ = -1.0f;
  cachedSurfaceArea_ = -1.0f;
  centroidValid_ = false;
}

}  // namespace msd_sim
