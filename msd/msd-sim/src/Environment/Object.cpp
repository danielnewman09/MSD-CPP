#include "msd-sim/src/Environment/Object.hpp"
#include <algorithm>

namespace msd_sim
{

Object::Object()
  : localVertices_{}, localFrame_{}, viewerFrameVertices_{}, sdlVertices_{}
{
}

Object::Object(const std::vector<Coordinate>& localVertices,
               const ReferenceFrame& frame)
  : localVertices_{localVertices}, localFrame_{frame}
{
  viewerFrameVertices_.resize(localVertices.size());
  sdlVertices_.resize(localVertices.size());

  // Initialize SDL vertices with default color
  for (auto& vertex : sdlVertices_)
  {
    vertex.color = color_;
  }
}

void Object::setLocalVertices(const std::vector<Coordinate>& vertices)
{
  localVertices_ = vertices;
  viewerFrameVertices_.resize(vertices.size());
  sdlVertices_.resize(vertices.size());

  // Initialize SDL vertices with current color
  for (auto& vertex : sdlVertices_)
  {
    vertex.color = color_;
  }
}

void Object::addLocalVertex(const Coordinate& vertex)
{
  localVertices_.push_back(vertex);
  viewerFrameVertices_.resize(localVertices_.size());
  sdlVertices_.resize(localVertices_.size());

  // Initialize new SDL vertex with current color
  sdlVertices_.back().color = color_;
}

ReferenceFrame& Object::getReferenceFrame()
{
  return localFrame_;
}

const ReferenceFrame& Object::getReferenceFrame() const
{
  return localFrame_;
}

size_t Object::getVertexCount() const
{
  return localVertices_.size();
}

void Object::transformToViewerFrame(const ReferenceFrame& viewerFrame)
{
  // Transform each vertex through the frame hierarchy:
  // Local -> Global -> Viewer
  for (size_t i = 0; i < localVertices_.size(); ++i)
  {
    // Step 1: Transform from object's local frame to global frame
    Coordinate globalCoord = localFrame_.localToGlobal(localVertices_[i]);

    // Step 2: Transform from global frame to viewer frame
    viewerFrameVertices_[i] = viewerFrame.globalToLocal(globalCoord);
  }

  // Update SDL vertices with projected 2D positions
  updateSDLVertices();
}

SDL_Vertex* Object::getSDLVertices()
{
  return sdlVertices_.data();
}

const SDL_Vertex* Object::getSDLVertices() const
{
  return sdlVertices_.data();
}

SDL_FPoint Object::projectTo2D(const Coordinate& coord,
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

void Object::setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
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

void Object::setProjectionParameters(float focalLength,
                                     float screenWidth,
                                     float screenHeight)
{
  focalLength_ = focalLength;
  screenWidth_ = screenWidth;
  screenHeight_ = screenHeight;
}

void Object::updateSDLVertices()
{
  // Project each 3D viewer frame coordinate to 2D screen space
  for (size_t i = 0; i < viewerFrameVertices_.size(); ++i)
  {
    sdlVertices_[i].position = projectTo2D(viewerFrameVertices_[i],
                                          focalLength_,
                                          screenWidth_,
                                          screenHeight_);
    // Color is already set, just update position
  }
}

void Object::render(SDL_Renderer* renderer) const
{
  // Render the object as filled triangles using SDL_RenderGeometry
  SDL_RenderGeometry(renderer,
                     nullptr,
                     sdlVertices_.data(),
                     sdlVertices_.size(),
                     nullptr,
                     0);
}

void Object::renderWireframe(SDL_Renderer* renderer,
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
  linePoints.reserve(sdlVertices_.size() * 2); // Each triangle has 3 edges

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
