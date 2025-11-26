#include "msd-sim/src/Environment/Polyhedron.hpp"
#include <algorithm>
#include <iostream>

namespace msd_sim
{

Polyhedron::Polyhedron()
  : localVertices_{}, localFrame_{}, viewerFrameVertices_{}, sdlVertices_{}
{
}

Polyhedron::Polyhedron(const std::vector<Coordinate>& localVertices,
                       const ReferenceFrame& frame)
  : localFrame_{frame}
{
  // Convert vector of Coordinates to 3xN matrix
  const size_t numVertices = localVertices.size();
  localVertices_.resize(3, numVertices);
  for (size_t i = 0; i < numVertices; ++i)
  {
    localVertices_.col(i) = localVertices[i];
  }

  viewerFrameVertices_.resize(3, numVertices);
  sdlVertices_.resize(numVertices);

  // Initialize SDL vertices with default color
  for (auto& vertex : sdlVertices_)
  {
    vertex.color = color_;
  }
}

void Polyhedron::setLocalVertices(const std::vector<Coordinate>& vertices)
{
  // Convert vector of Coordinates to 3xN matrix
  const size_t numVertices = vertices.size();
  localVertices_.resize(3, numVertices);
  for (size_t i = 0; i < numVertices; ++i)
  {
    localVertices_.col(i) = vertices[i];
  }

  viewerFrameVertices_.resize(3, numVertices);
  sdlVertices_.resize(numVertices);

  // Initialize SDL vertices with current color
  for (auto& vertex : sdlVertices_)
  {
    vertex.color = color_;
  }
}

void Polyhedron::addLocalVertex(const Coordinate& vertex)
{
  // Append a column to the matrix
  const Eigen::Index oldCols = localVertices_.cols();
  localVertices_.conservativeResize(3, oldCols + 1);
  localVertices_.col(oldCols) = vertex;

  viewerFrameVertices_.conservativeResize(3, oldCols + 1);
  sdlVertices_.resize(oldCols + 1);

  // Initialize new SDL vertex with current color
  sdlVertices_.back().color = color_;
}

ReferenceFrame& Polyhedron::getReferenceFrame()
{
  return localFrame_;
}

const ReferenceFrame& Polyhedron::getReferenceFrame() const
{
  return localFrame_;
}

size_t Polyhedron::getVertexCount() const
{
  return localVertices_.cols();
}

void Polyhedron::transformToViewerFrame(const ReferenceFrame& viewerFrame)
{
  // Precompute combined transform: local -> global -> viewer
  // Combined rotation: R_viewer^T * R_local
  const Eigen::Matrix3d combinedRotation =
    viewerFrame.getRotation().transpose() * localFrame_.getRotation();

  // Combined translation: R_viewer^T * (origin_local - origin_viewer)
  const Eigen::Vector3d combinedTranslation =
    viewerFrame.getRotation().transpose() *
    (localFrame_.getOrigin() - viewerFrame.getOrigin());

  // Apply combined transform in one operation (no copy of localVertices_)
  viewerFrameVertices_.noalias() = combinedRotation * localVertices_;
  viewerFrameVertices_.colwise() += combinedTranslation;

  // Update SDL vertices with projected 2D positions
  updateSDLVertices();
}

SDL_Vertex* Polyhedron::getSDLVertices()
{
  return sdlVertices_.data();
}

const SDL_Vertex* Polyhedron::getSDLVertices() const
{
  return sdlVertices_.data();
}

SDL_FPoint Polyhedron::projectTo2D(const Coordinate& coord,
                                   float focalLength,
                                   float screenWidth,
                                   float screenHeight)
{
  SDL_FPoint result;

  // Aerospace convention: X is forward, Y is right, Z is up
  // For perspective projection, we project onto the plane perpendicular to X
  // using X as the depth coordinate

  // Avoid division by zero - clamp x to minimum distance
  float depth = std::max(coord.x(), 0.1);

  // Perspective projection:
  // - Y maps to horizontal screen position (right)
  // - Z maps to vertical screen position (up, but screen Y is down, so negate)
  result.x = (coord.y() * focalLength) / depth + (screenWidth / 2.0f);
  result.y = (-coord.z() * focalLength) / depth + (screenHeight / 2.0f);

  return result;
}

void Polyhedron::setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
  color_.r = r;
  color_.g = g;
  color_.b = b;
  color_.a = a;

  // Update all existing SDL vertices with new color
  for (auto& vertex : sdlVertices_)
  {
    vertex.color = color_;
  }
}

void Polyhedron::setProjectionParameters(float focalLength,
                                         float screenWidth,
                                         float screenHeight)
{
  focalLength_ = focalLength;
  screenWidth_ = screenWidth;
  screenHeight_ = screenHeight;
}

void Polyhedron::updateSDLVertices()
{
  const float halfWidth = screenWidth_ / 2.0f;
  const float halfHeight = screenHeight_ / 2.0f;

  // Project each 3D viewer frame coordinate to 2D screen space
  for (Eigen::Index i = 0; i < viewerFrameVertices_.cols(); ++i)
  {
    // Avoid division by zero - clamp x (depth) to minimum distance
    const float depth = std::max(static_cast<float>(viewerFrameVertices_(0, i)), 0.1f);

    // Perspective projection (aerospace: X-forward, Y-right, Z-up)
    sdlVertices_[i].position.x =
      static_cast<float>(viewerFrameVertices_(1, i)) * focalLength_ / depth + halfWidth;
    sdlVertices_[i].position.y =
      -static_cast<float>(viewerFrameVertices_(2, i)) * focalLength_ / depth + halfHeight;
    // Color is already set, just update position
  }
}

void Polyhedron::render(SDL_Renderer* renderer) const
{
  // Render the object as filled triangles using SDL_RenderGeometry
  SDL_RenderGeometry(
    renderer, nullptr, sdlVertices_.data(), sdlVertices_.size(), nullptr, 0);
}

void Polyhedron::renderWireframe(SDL_Renderer* renderer,
                                 uint8_t r,
                                 uint8_t g,
                                 uint8_t b,
                                 uint8_t a) const
{
  if (sdlVertices_.empty())
  {
    return;
  }

  // Set wireframe color
  SDL_SetRenderDrawColor(renderer, r, g, b, a);

  // Extract edges from triangles
  // For each triangle (3 vertices), draw 3 edges
  std::vector<SDL_FPoint> linePoints;
  linePoints.reserve(sdlVertices_.size() * 2);  // Each triangle has 3 edges

  for (size_t i = 0; i + 2 < sdlVertices_.size(); i += 3)
  {
    // Triangle vertices: i, i+1, i+2
    // Edge 1: i -> i+1
    linePoints.push_back(sdlVertices_[i].position);
    linePoints.push_back(sdlVertices_[i + 1].position);

    // Edge 2: i+1 -> i+2
    linePoints.push_back(sdlVertices_[i + 1].position);
    linePoints.push_back(sdlVertices_[i + 2].position);

    // Edge 3: i+2 -> i
    linePoints.push_back(sdlVertices_[i + 2].position);
    linePoints.push_back(sdlVertices_[i].position);
  }

  // Render all lines
  SDL_RenderLines(renderer, linePoints.data(), linePoints.size());
}

}  // namespace msd_sim
