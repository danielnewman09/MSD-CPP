// Ticket: 0005_camera_controller_sim
// Design: docs/designs/0005_camera_controller_sim/design.md

#include "msd-sim/src/Environment/MotionController.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <chrono>
#include "msd-sim/src/Agent/InputCommands.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"

namespace msd_sim
{

MotionController::MotionController(double rotSpeed, double moveSpeed)
  : rotSpeed_{rotSpeed}, moveSpeed_{moveSpeed}
{
}

void MotionController::updateTransform(
  ReferenceFrame& frame,
  const InputCommands& commands,
  std::chrono::milliseconds deltaTime) const
{
  // Convert deltaTime to seconds for speed calculations
  const double deltaSeconds = static_cast<double>(deltaTime.count()) / 1000.0;
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
    newOrigin -= frame.localToGlobal(Eigen::Vector3d{0, 0, scaledMoveSpeed});
  }
  if (commands.moveBackward)
  {
    // Move backward in local Z direction
    newOrigin += frame.localToGlobal(Eigen::Vector3d{0, 0, scaledMoveSpeed});
  }
  if (commands.moveLeft)
  {
    // Move left in local X direction
    newOrigin -= frame.localToGlobal(Eigen::Vector3d{scaledMoveSpeed, 0, 0});
  }
  if (commands.moveRight)
  {
    // Move right in local X direction
    newOrigin += frame.localToGlobal(Eigen::Vector3d{scaledMoveSpeed, 0, 0});
  }
  if (commands.moveUp)
  {
    // Move up in local Y direction
    newOrigin += frame.localToGlobal(Eigen::Vector3d{0, scaledMoveSpeed, 0});
  }
  if (commands.moveDown)
  {
    // Move down in local Y direction
    newOrigin -= frame.localToGlobal(Eigen::Vector3d{0, scaledMoveSpeed, 0});
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
  Eigen::Quaterniond const q =
    Eigen::AngleAxisd{angular.yaw(), Eigen::Vector3d::UnitZ()} *
    Eigen::AngleAxisd{angular.pitch(), Eigen::Vector3d::UnitY()} *
    Eigen::AngleAxisd{angular.roll(), Eigen::Vector3d::UnitX()};
  frame.setQuaternion(q);
}

}  // namespace msd_sim
