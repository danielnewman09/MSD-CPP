// Ticket: 0005_camera_controller_sim
// Design: docs/designs/0005_camera_controller_sim/design.md

#ifndef MOTION_CONTROLLER_HPP
#define MOTION_CONTROLLER_HPP

#include <chrono>

#include "msd-sim/src/Agent/InputCommands.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"

namespace msd_sim
{

/**
 * @brief Encapsulates 3D motion control logic for simulation objects
 *
 * Translates input commands into ReferenceFrame updates (translation and
 * rotation). Replaces msd-gui::CameraController with simulation-appropriate
 * implementation.
 *
 * Supports frame-rate independent movement through delta-time scaling.
 * Movement is applied in the local coordinate frame of the provided
 * ReferenceFrame.
 *
 * @see docs/designs/0005_camera_controller_sim/0005_camera_controller_sim.puml
 * @ticket 0005_camera_controller_sim
 *
 * Thread safety: Not thread-safe (modifies ReferenceFrame in place)
 */
class MotionController
{
public:
  /**
   * @brief Construct a motion controller
   * @param rotSpeed Rotation speed in radians per second (default: 0.05 rad/s)
   * @param moveSpeed Movement speed in units per second (default: 0.1 m/s)
   */
  explicit MotionController(double rotSpeed = 0.05, double moveSpeed = 0.1);

  ~MotionController() = default;

  MotionController(const MotionController&) = default;
  MotionController& operator=(const MotionController&) = default;
  MotionController(MotionController&&) noexcept = default;
  MotionController& operator=(MotionController&&) noexcept = default;

  /**
   * @brief Update a ReferenceFrame based on input commands
   * @param frame ReferenceFrame to update (non-owning reference)
   * @param commands Input commands (boolean flags)
   * @param deltaTime Time elapsed since last update (for frame-rate
   * independence)
   *
   * Applies linear movement in local frame:
   * - moveForward: -Z local direction
   * - moveBackward: +Z local direction
   * - moveLeft: -X local direction
   * - moveRight: +X local direction
   * - moveUp: +Y local direction
   * - moveDown: -Y local direction
   *
   * Applies rotation:
   * - pitchUp: Increase pitch angle
   * - pitchDown: Decrease pitch angle
   * - yawLeft: Increase yaw angle
   * - yawRight: Decrease yaw angle
   *
   * All movements scaled by moveSpeed * sensitivity and deltaTime.
   * All rotations scaled by rotSpeed and deltaTime.
   */
  void updateTransform(ReferenceFrame& frame,
                       const InputCommands& commands,
                       std::chrono::milliseconds deltaTime);

  /**
   * @brief Set movement speed
   * @param speed Units per second
   */
  void setMoveSpeed(double speed)
  {
    moveSpeed_ = speed;
  }

  /**
   * @brief Set rotation speed
   * @param speed Radians per second
   */
  void setRotationSpeed(double speed)
  {
    rotSpeed_ = speed;
  }

  /**
   * @brief Set input sensitivity multiplier
   * @param sensitivity Multiplier for input (default: 1.0)
   */
  void setSensitivity(double sensitivity)
  {
    sensitivity_ = sensitivity;
  }

  /**
   * @brief Get movement speed
   * @return Units per second
   */
  double getMoveSpeed() const
  {
    return moveSpeed_;
  }

  /**
   * @brief Get rotation speed
   * @return Radians per second
   */
  double getRotationSpeed() const
  {
    return rotSpeed_;
  }

  /**
   * @brief Get input sensitivity
   * @return Sensitivity multiplier
   */
  double getSensitivity() const
  {
    return sensitivity_;
  }

private:
  double rotSpeed_;     // Radians per second
  double moveSpeed_;    // Units per second
  double sensitivity_;  // Input multiplier
};

}  // namespace msd_sim

#endif  // MOTION_CONTROLLER_HPP
