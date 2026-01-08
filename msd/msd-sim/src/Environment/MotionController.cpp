// Ticket: 0005_camera_controller_sim
// Design: docs/designs/0005_camera_controller_sim/design.md

#include "msd-sim/src/Environment/MotionController.hpp"

#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim
{

MotionController::MotionController(Angle rotSpeed, float moveSpeed)
  : rotSpeed_{rotSpeed}, moveSpeed_{moveSpeed}, sensitivity_{1.0f}
{
}

void MotionController::updateTransform(ReferenceFrame& frame,
                                       const InputCommands& commands,
                                       std::chrono::milliseconds deltaTime)
{
  // Convert deltaTime to seconds for speed calculations
  const double deltaSeconds = deltaTime.count() / 1000.0;
  const float scaledMoveSpeed = moveSpeed_ * sensitivity_ * static_cast<float>(deltaSeconds);
  const Angle scaledRotSpeed = rotSpeed_ * static_cast<float>(deltaSeconds);

  auto newOrigin = frame.getOrigin();
  auto& eulerAngles = frame.getEulerAngles();

  // Linear movement in local frame
  // Note: Camera's local Z is forward (negative Z is into the screen in camera space)
  if (commands.moveForward)
  {
    // Move forward in local Z direction
    newOrigin -= frame.localToGlobalRelative(Coordinate{0, 0, scaledMoveSpeed});
  }
  if (commands.moveBackward)
  {
    // Move backward in local Z direction
    newOrigin += frame.localToGlobalRelative(Coordinate{0, 0, scaledMoveSpeed});
  }
  if (commands.moveLeft)
  {
    // Move left in local X direction
    newOrigin -= frame.localToGlobalRelative(Coordinate{scaledMoveSpeed, 0, 0});
  }
  if (commands.moveRight)
  {
    // Move right in local X direction
    newOrigin += frame.localToGlobalRelative(Coordinate{scaledMoveSpeed, 0, 0});
  }
  if (commands.moveUp)
  {
    // Move up in local Y direction
    newOrigin += frame.localToGlobalRelative(Coordinate{0, scaledMoveSpeed, 0});
  }
  if (commands.moveDown)
  {
    // Move down in local Y direction
    newOrigin -= frame.localToGlobalRelative(Coordinate{0, scaledMoveSpeed, 0});
  }

  // Apply movement
  frame.setOrigin(newOrigin);

  // Rotation
  if (commands.pitchUp)
  {
    // Pitch up (look up)
    eulerAngles.pitch += scaledRotSpeed;
  }
  if (commands.pitchDown)
  {
    // Pitch down (look down)
    eulerAngles.pitch -= scaledRotSpeed;
  }
  if (commands.yawLeft)
  {
    // Yaw left (turn left)
    eulerAngles.yaw += scaledRotSpeed;
  }
  if (commands.yawRight)
  {
    // Yaw right (turn right)
    eulerAngles.yaw -= scaledRotSpeed;
  }

  // Update rotation matrix in reference frame
  frame.setRotation(eulerAngles);
}

}  // namespace msd_sim
