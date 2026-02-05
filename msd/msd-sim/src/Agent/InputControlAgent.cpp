// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#include "msd-sim/src/Agent/InputControlAgent.hpp"
#include <Eigen/src/Core/Matrix.h>
#include "msd-sim/src/DataTypes/AngularRate.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

InputControlAgent::InputControlAgent(double maxSpeed, double maxAngularSpeed)
  : maxSpeed_{maxSpeed}, maxAngularSpeed_{maxAngularSpeed}
{
}

InertialState InputControlAgent::updateState(const InertialState& currentState) const
{
  InertialState newState = currentState;

  // Calculate linear velocity based on input commands
  Eigen::Vector3d velocity{0, 0, 0};

  if (inputCommands_.moveForward)
  {
    velocity.x() += maxSpeed_;
  }
  if (inputCommands_.moveBackward)
  {
    velocity.x() -= maxSpeed_;
  }
  if (inputCommands_.moveRight)
  {
    velocity.y() += maxSpeed_;
  }
  if (inputCommands_.moveLeft)
  {
    velocity.y() -= maxSpeed_;
  }
  if (inputCommands_.moveUp)
  {
    velocity.z() += maxSpeed_;
  }
  if (inputCommands_.moveDown)
  {
    velocity.z() -= maxSpeed_;
  }

  newState.velocity = velocity;

  // Calculate angular velocity based on input commands
  // Note: angularVelocity is now AngularRate (vector form) per ticket 0024
  AngularRate angularVelocity{0, 0, 0};

  if (inputCommands_.pitchUp)
  {
    angularVelocity.pitch() =
      maxAngularSpeed_;  // pitch around Y-axis (aerospace convention)
  }
  if (inputCommands_.pitchDown)
  {
    angularVelocity.pitch() = -maxAngularSpeed_;
  }
  if (inputCommands_.rollLeft)
  {
    angularVelocity.roll() = maxAngularSpeed_;  // roll around X-axis
  }
  if (inputCommands_.rollRight)
  {
    angularVelocity.roll() = -maxAngularSpeed_;
  }
  if (inputCommands_.yawLeft)
  {
    angularVelocity.yaw() = maxAngularSpeed_;  // yaw around Z-axis
  }
  if (inputCommands_.yawRight)
  {
    angularVelocity.yaw() = -maxAngularSpeed_;
  }

  // Convert angular velocity to quaternion rate and store
  newState.setAngularVelocity(angularVelocity);
  // Ticket: 0030_lagrangian_quaternion_physics

  // Note: Jump command not yet implemented (requires physics integration)
  // TODO: Implement jump as impulse when physics component is present

  return newState;
}

}  // namespace msd_sim
