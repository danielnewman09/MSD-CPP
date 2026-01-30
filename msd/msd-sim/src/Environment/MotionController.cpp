// Ticket: 0005_camera_controller_sim
// Design: docs/designs/0005_camera_controller_sim/design.md

#include "msd-sim/src/Environment/MotionController.hpp"

#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim
{

MotionController::MotionController(double rotSpeed, double moveSpeed)
  : rotSpeed_{rotSpeed}, moveSpeed_{moveSpeed}, sensitivity_{1.0}
{
}

void MotionController::updateTransform(ReferenceFrame& frame,
                                       const InputCommands& commands,
                                       std::chrono::milliseconds deltaTime)
{
  // Convert deltaTime to seconds for speed calculations
  const double deltaSeconds = deltaTime.count() / 1000.0;
  const double scaledMoveSpeed = moveSpeed_ * sensitivity_ * deltaSeconds;
  const double scaledRotSpeed = rotSpeed_ * deltaSeconds;

  auto newOrigin = frame.getOrigin();
  auto& angular = frame.getAngularCoordinate();

  // Linear movement in local frame
  // Note: Camera's local Z is forward (negative Z is into the screen in camera
  // space)
  if (commands.moveForward)
  {
    // Move forward in local Z direction
    newOrigin -= frame.localToGlobal(CoordinateRate{0, 0, scaledMoveSpeed});
  }
  if (commands.moveBackward)
  {
    // Move backward in local Z direction
    newOrigin += frame.localToGlobal(CoordinateRate{0, 0, scaledMoveSpeed});
  }
  if (commands.moveLeft)
  {
    // Move left in local X direction
    newOrigin -= frame.localToGlobal(CoordinateRate{scaledMoveSpeed, 0, 0});
  }
  if (commands.moveRight)
  {
    // Move right in local X direction
    newOrigin += frame.localToGlobal(CoordinateRate{scaledMoveSpeed, 0, 0});
  }
  if (commands.moveUp)
  {
    // Move up in local Y direction
    newOrigin += frame.localToGlobal(CoordinateRate{0, scaledMoveSpeed, 0});
  }
  if (commands.moveDown)
  {
    // Move down in local Y direction
    newOrigin -= frame.localToGlobal(CoordinateRate{0, scaledMoveSpeed, 0});
  }

  // Apply movement
  frame.setOrigin(newOrigin);

  // Rotation - modify angular coordinate directly
  if (commands.pitchUp)
  {
    // Pitch up (look up)
    angular.setPitch(angular.pitch() + scaledRotSpeed);
  }
  if (commands.pitchDown)
  {
    // Pitch down (look down)
    angular.setPitch(angular.pitch() - scaledRotSpeed);
  }
  if (commands.yawLeft)
  {
    // Yaw left (turn left)
    angular.setYaw(angular.yaw() + scaledRotSpeed);
  }
  if (commands.yawRight)
  {
    // Yaw right (turn right)
    angular.setYaw(angular.yaw() - scaledRotSpeed);
  }

  // Update rotation from Euler angles via quaternion (ZYX convention)
  Eigen::Quaterniond q =
      Eigen::AngleAxisd{angular.yaw(), Eigen::Vector3d::UnitZ()} *
      Eigen::AngleAxisd{angular.pitch(), Eigen::Vector3d::UnitY()} *
      Eigen::AngleAxisd{angular.roll(), Eigen::Vector3d::UnitX()};
  frame.setQuaternion(q);
}

}  // namespace msd_sim
