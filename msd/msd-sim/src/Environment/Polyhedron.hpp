#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <SDL3/SDL.h>
#include <Eigen/Dense>
#include <vector>
#include "msd-sim/src/Environment/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"

namespace msd_sim
{

/**
 * @brief Represents a 3D object with geometry defined in its local frame
 *
 * An Polyhedron maintains:
 * - A set of vertices (Coordinates) defined relative to its own local frame
 * - A ReferenceFrame that positions/orients the object in the global frame
 * - Efficient transformation to viewer frame for rendering
 *
 * Frame hierarchy:
 * 1. Local frame: Polyhedron vertices are constant in this frame
 * 2. Global frame: Polyhedron's local frame is positioned within global frame
 * 3. Viewer frame: Camera/observer frame for rendering
 *
 * The class provides zero-copy access to transformed vertices for SDL
 * rendering.
 */
class Polyhedron
{
public:
  /**
   * @brief Default constructor - creates an empty object at origin
   */
  Polyhedron();

  /**
   * @brief Constructor with local vertices and reference frame
   * @param localVertices Vertices defined in the object's local frame
   * @param frame The object's reference frame in global coordinates
   */
  Polyhedron(const std::vector<Coordinate>& localVertices,
         const ReferenceFrame& frame);

  /**
   * @brief Set the vertices in the object's local frame
   * @param vertices Vertex positions in local coordinates
   */
  void setLocalVertices(const std::vector<Coordinate>& vertices);

  /**
   * @brief Add a single vertex to the object's local geometry
   * @param vertex Vertex position in local coordinates
   */
  void addLocalVertex(const Coordinate& vertex);

  /**
   * @brief Get the object's reference frame (position/orientation in global)
   * @return Reference to the object's frame
   */
  ReferenceFrame& getReferenceFrame();

  /**
   * @brief Get the object's reference frame (const version)
   * @return Const reference to the object's frame
   */
  const ReferenceFrame& getReferenceFrame() const;

  /**
   * @brief Get the number of vertices
   * @return Number of vertices in the object
   */
  size_t getVertexCount() const;

  /**
   * @brief Transform all vertices to viewer frame and prepare for rendering
   *
   * This method transforms vertices through the frame hierarchy:
   * Local -> Global -> Viewer
   *
   * The transformation is:
   * 1. localToGlobal: Apply object's reference frame transformation
   * 2. globalToViewer: Apply viewer's frame transformation
   *
   * @param viewerFrame The camera/observer's reference frame
   */
  void transformToViewerFrame(const ReferenceFrame& viewerFrame);

  /**
   * @brief Get pointer to SDL_Vertex array for rendering
   *
   * This provides zero-copy access to the transformed vertices.
   * The vertices are already in viewer frame coordinates (2D projected).
   *
   * IMPORTANT: Must call transformToViewerFrame() before calling this.
   *
   * @return Pointer to SDL_Vertex array suitable for SDL_RenderGeometry
   */
  SDL_Vertex* getSDLVertices();

  /**
   * @brief Get const pointer to SDL_Vertex array for rendering
   * @return Const pointer to SDL_Vertex array
   */
  const SDL_Vertex* getSDLVertices() const;

  /**
   * @brief Project a 3D coordinate to 2D screen space
   *
   * Uses aerospace convention (X-forward, Y-right, Z-up):
   * - X is the depth (forward direction)
   * - Y projects to horizontal screen position (right)
   * - Z projects to vertical screen position (up, negated for screen coords)
   *
   * Perspective projection:
   * x_screen = (y * focal_length) / x + center_x
   * y_screen = (-z * focal_length) / x + center_y
   *
   * @param coord 3D coordinate in viewer frame (X-forward, Y-right, Z-up)
   * @param focalLength Perspective focal length (higher = less perspective)
   * @param screenWidth Width of the rendering viewport
   * @param screenHeight Height of the rendering viewport
   * @return 2D screen position (x, y)
   */
  static SDL_FPoint projectTo2D(const Coordinate& coord,
                                float focalLength,
                                float screenWidth,
                                float screenHeight);

  /**
   * @brief Set the color for all vertices
   * @param r Red component (0-255)
   * @param g Green component (0-255)
   * @param b Blue component (0-255)
   * @param a Alpha component (0-255)
   */
  void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);

  /**
   * @brief Set the perspective parameters for projection
   * @param focalLength Distance from viewer to projection plane
   * @param screenWidth Width of viewport in pixels
   * @param screenHeight Height of viewport in pixels
   */
  void setProjectionParameters(float focalLength,
                               float screenWidth,
                               float screenHeight);

  /**
   * @brief Render the object as filled triangles
   * @param renderer SDL renderer to draw with
   */
  void render(SDL_Renderer* renderer) const;

  /**
   * @brief Render the object as a wireframe
   *
   * Extracts unique edges from the triangulated mesh and renders them as lines.
   * This assumes vertices represent triangles (groups of 3).
   *
   * @param renderer SDL renderer to draw with
   * @param r Red component for wireframe color (0-255)
   * @param g Green component for wireframe color (0-255)
   * @param b Blue component for wireframe color (0-255)
   * @param a Alpha component for wireframe color (0-255)
   */
  void renderWireframe(SDL_Renderer* renderer,
                      uint8_t r = 255,
                      uint8_t g = 255,
                      uint8_t b = 255,
                      uint8_t a = 255) const;

private:
  //! Vertices defined in the object's local reference frame (constant)
  //! Stored as 3xN matrix for efficient batch transformations
  Eigen::Matrix3Xd localVertices_;

  //! The object's reference frame within the global frame
  ReferenceFrame localFrame_;

  //! Transformed vertices in viewer frame (updated by transformToViewerFrame)
  //! Stored as 3xN matrix for efficient batch transformations
  Eigen::Matrix3Xd viewerFrameVertices_;

  //! SDL vertex array for rendering (positions projected to 2D)
  std::vector<SDL_Vertex> sdlVertices_;

  //! Cached line points for wireframe rendering (reused every frame)
  mutable std::vector<SDL_FPoint> wireframeLinePoints_;

  //! Projection parameters
  float focalLength_{500.0f};
  float screenWidth_{640.0f};
  float screenHeight_{480.0f};

  //! Current color for all vertices
  SDL_FColor color_{255, 255, 255, 255};

  /**
   * @brief Update SDL vertex buffer from viewer frame coordinates
   * Projects 3D viewer coordinates to 2D screen space
   */
  void updateSDLVertices();
};

}  // namespace msd_sim

#endif  // OBJECT_HPP
