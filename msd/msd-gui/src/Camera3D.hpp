// Ticket: 0005_camera_controller_sim
// Design: docs/designs/0005_camera_controller_sim/design.md

#ifndef CAMERA_3D_HPP
#define CAMERA_3D_HPP

#include <Eigen/Dense>
#include <functional>
#include <msd-sim/src/Environment/ReferenceFrame.hpp>

namespace msd_gui
{

/**
 * @brief A 3D camera for rendering with perspective projection
 *
 * This class references a ReferenceFrame and provides camera-specific functionality
 * including view matrix and projection matrix computation for 3D rendering.
 * It generates the Model-View-Projection (MVP) matrix needed by shaders.
 *
 * The camera uses a right-handed coordinate system with:
 * - X: right
 * - Y: up
 * - Z: forward (out of screen, opposite viewing direction)
 *
 * BREAKING CHANGE (Ticket 0005_camera_controller_sim):
 * Camera now takes a non-owning reference to ReferenceFrame instead of owning one.
 * This allows simulation logic to control camera position through the referenced frame.
 *
 * @see docs/designs/0005_camera_controller_sim/0005_camera_controller_sim.puml
 * @ticket 0005_camera_controller_sim
 */
class Camera3D
{
public:
  /**
   * @brief Constructor with reference frame, field of view, and aspect ratio
   * @param referenceFrame Reference to external ReferenceFrame (non-owning)
   * @param fovDegrees Vertical field of view in degrees (default: 60)
   * @param aspectRatio Width/height ratio (default: 16/9)
   * @param nearPlane Near clipping plane distance (default: 0.1)
   * @param farPlane Far clipping plane distance (default: 100.0)
   *
   * NOTE: The provided ReferenceFrame must outlive this Camera3D instance.
   * Typically the ReferenceFrame is owned by a Platform's visual Object.
   */
  explicit Camera3D(msd_sim::ReferenceFrame& referenceFrame,
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
    return frame_.get();
  }

  /**
   * @brief Get the underlying reference frame (const version)
   * @return Const reference to the camera's reference frame
   */
  const msd_sim::ReferenceFrame& getReferenceFrame() const
  {
    return frame_.get();
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
  std::reference_wrapper<msd_sim::ReferenceFrame> frame_;  ///< Non-owning reference to camera's reference frame
  float fovRadians_;                                       ///< Vertical field of view in radians
  float aspectRatio_;                                      ///< Width/height ratio
  float nearPlane_;                                        ///< Near clipping plane distance
  float farPlane_;                                         ///< Far clipping plane distance
};

}  // namespace msd_gui

#endif  // CAMERA_3D_HPP
