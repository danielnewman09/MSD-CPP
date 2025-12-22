#ifndef CAMERA_3D_HPP
#define CAMERA_3D_HPP

#include <Eigen/Dense>
#include <msd-sim/src/Environment/ReferenceFrame.hpp>

namespace msd_gui
{

/**
 * @brief A 3D camera for rendering with perspective projection
 *
 * This class wraps a ReferenceFrame and provides camera-specific functionality
 * including view matrix and projection matrix computation for 3D rendering.
 * It generates the Model-View-Projection (MVP) matrix needed by shaders.
 *
 * The camera uses a right-handed coordinate system with:
 * - X: right
 * - Y: up
 * - Z: forward (out of screen, opposite viewing direction)
 */
class Camera3D
{
public:
  /**
   * @brief Constructor with position, field of view, and aspect ratio
   * @param position Initial camera position in world coordinates
   * @param fovDegrees Vertical field of view in degrees (default: 60)
   * @param aspectRatio Width/height ratio (default: 16/9)
   * @param nearPlane Near clipping plane distance (default: 0.1)
   * @param farPlane Far clipping plane distance (default: 100.0)
   */
  explicit Camera3D(const msd_sim::Coordinate& position,
                    float fovDegrees = 60.0f,
                    float aspectRatio = 16.0f / 9.0f,
                    float nearPlane = 0.1f,
                    float farPlane = 100.0f);

  /**
   * @brief Get the underlying reference frame
   * @return Reference to the camera's reference frame
   */
  msd_sim::ReferenceFrame& getReferenceFrame()
  {
    return frame_;
  }

  /**
   * @brief Get the underlying reference frame (const version)
   * @return Const reference to the camera's reference frame
   */
  const msd_sim::ReferenceFrame& getReferenceFrame() const
  {
    return frame_;
  }

  /**
   * @brief Set the aspect ratio (width/height)
   * @param aspectRatio New aspect ratio
   */
  void setAspectRatio(float aspectRatio);

  /**
   * @brief Set the field of view
   * @param fovDegrees Vertical field of view in degrees
   */
  void setFieldOfView(float fovDegrees);

  /**
   * @brief Set the near and far clipping planes
   * @param nearPlane Near clipping plane distance
   * @param farPlane Far clipping plane distance
   */
  void setClippingPlanes(float nearPlane, float farPlane);

  /**
   * @brief Get the view matrix (transforms from world space to camera space)
   * @return 4x4 view matrix in single precision (column-major)
   */
  Eigen::Matrix4f getViewMatrix() const;

  /**
   * @brief Get the projection matrix (perspective projection)
   * @return 4x4 projection matrix in single precision (column-major)
   */
  Eigen::Matrix4f getProjectionMatrix() const;

  /**
   * @brief Get the combined Model-View-Projection matrix
   * @param modelMatrix Optional model matrix (default: identity)
   * @return 4x4 MVP matrix in single precision (column-major)
   */
  Eigen::Matrix4f getMVPMatrix(
    const Eigen::Matrix4f& modelMatrix = Eigen::Matrix4f::Identity()) const;

private:
  msd_sim::ReferenceFrame frame_;  ///< Camera's reference frame
  float fovRadians_;               ///< Vertical field of view in radians
  float aspectRatio_;              ///< Width/height ratio
  float nearPlane_;                ///< Near clipping plane distance
  float farPlane_;                 ///< Far clipping plane distance
};

}  // namespace msd_gui

#endif  // CAMERA_3D_HPP
