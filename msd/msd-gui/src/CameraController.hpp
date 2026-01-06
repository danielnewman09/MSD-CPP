// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#ifndef CAMERA_CONTROLLER_HPP
#define CAMERA_CONTROLLER_HPP

#include <chrono>

#include "msd-gui/src/Camera3D.hpp"
#include "msd-gui/src/InputState.hpp"
#include "msd-sim/src/Environment/Angle.hpp"

namespace msd_gui
{

/**
 * @brief Encapsulates camera movement logic based on InputState
 *
 * Separates camera control logic from SDLApp (single responsibility).
 * Reads InputState but doesn't modify it (InputHandler owns state).
 *
 * Camera movement mapping:
 * - W/S: Move forward/backward in camera's local Z direction
 * - A/D: Move left/right in camera's local X direction
 * - Q/E: Move up/down in camera's local Y direction
 * - Arrow Up/Down: Pitch camera
 * - Arrow Left/Right: Yaw camera
 *
 * Thread safety: Not thread-safe
 */
class CameraController
{
public:
  /**
   * @brief Construct a camera controller
   * @param camera Camera to control (non-owning reference)
   * @param moveSpeed Movement speed in units per frame (default: 0.1)
   * @param rotSpeed Rotation speed in radians per frame (default: 0.05)
   */
  explicit CameraController(Camera3D& camera,
                            float moveSpeed = 0.1f,
                            msd_sim::Angle rotSpeed = msd_sim::Angle::fromRadians(0.05));

  CameraController(const CameraController&) = delete;
  CameraController& operator=(const CameraController&) = delete;
  CameraController(CameraController&&) noexcept = default;
  CameraController& operator=(CameraController&&) noexcept = default;
  ~CameraController() = default;

  /**
   * @brief Update camera from input state
   * @param inputState Current input state
   * @param deltaTime Time elapsed since last update (for frame-rate independence)
   *
   * Reads the input state and applies camera movement/rotation accordingly.
   * Does not modify the input state.
   */
  void updateFromInput(const InputState& inputState,
                       std::chrono::milliseconds deltaTime);

  /**
   * @brief Set movement speed
   * @param speed Units per frame
   */
  void setMoveSpeed(float speed) { moveSpeed_ = speed; }

  /**
   * @brief Set rotation speed
   * @param speed Radians per frame
   */
  void setRotationSpeed(msd_sim::Angle speed) { rotSpeed_ = speed; }

  /**
   * @brief Set input sensitivity multiplier
   * @param sensitivity Multiplier for input (default: 1.0)
   */
  void setSensitivity(float sensitivity) { sensitivity_ = sensitivity; }

  /**
   * @brief Get movement speed
   * @return Units per frame
   */
  float getMoveSpeed() const { return moveSpeed_; }

  /**
   * @brief Get rotation speed
   * @return Radians per frame
   */
  msd_sim::Angle getRotationSpeed() const { return rotSpeed_; }

private:
  Camera3D& camera_;                     // Non-owning reference
  float moveSpeed_{0.1f};                // Units per frame
  msd_sim::Angle rotSpeed_{0.05};        // Radians per frame
  float sensitivity_{1.0f};              // Input multiplier
};

}  // namespace msd_gui

#endif  // CAMERA_CONTROLLER_HPP
