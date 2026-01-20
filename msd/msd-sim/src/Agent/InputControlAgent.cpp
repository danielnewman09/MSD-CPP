// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#include "msd-sim/src/Agent/InputControlAgent.hpp"

namespace msd_sim
{

InputControlAgent::InputControlAgent(double maxSpeed, Angle maxAngularSpeed)
  : maxSpeed_{maxSpeed}, maxAngularSpeed_{maxAngularSpeed}
{
}

InertialState InputControlAgent::updateState(const InertialState& currentState)
{
  InertialState newState = currentState;

  // Calculate linear velocity based on input commands
  Coordinate velocity{0, 0, 0};

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
  // Note: angularVelocity is now Coordinate (vector form) per ticket 0023a
  Coordinate angularVelocity{0, 0, 0};

  if (inputCommands_.pitchUp)
  {
    angularVelocity.x() = maxAngularSpeed_.getRad();  // pitch around X-axis
  }
  if (inputCommands_.pitchDown)
  {
    angularVelocity.x() = -maxAngularSpeed_.getRad();
  }
  if (inputCommands_.rollLeft)
  {
    angularVelocity.y() = maxAngularSpeed_.getRad();  // roll around Y-axis
  }
  if (inputCommands_.rollRight)
  {
    angularVelocity.y() = -maxAngularSpeed_.getRad();
  }
  if (inputCommands_.yawLeft)
  {
    angularVelocity.z() = maxAngularSpeed_.getRad();  // yaw around Z-axis
  }
  if (inputCommands_.yawRight)
  {
    angularVelocity.z() = -maxAngularSpeed_.getRad();
  }

  newState.angularVelocity = angularVelocity;

  // Note: Jump command not yet implemented (requires physics integration)
  // TODO: Implement jump as impulse when physics component is present

  return newState;
}

}  // namespace msd_sim
