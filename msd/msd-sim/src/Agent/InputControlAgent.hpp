// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#ifndef INPUT_CONTROL_AGENT_HPP
#define INPUT_CONTROL_AGENT_HPP

#include "msd-sim/src/Agent/BaseAgent.hpp"
#include "msd-sim/src/Agent/InputCommands.hpp"
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/**
 * @brief Agent implementation that translates InputCommands into InertialState
 * updates
 *
 * Implements the BaseAgent interface to fit cleanly into the simulation
 * architecture. Separates input commands from state update logic.
 *
 * Usage flow:
 * 1. GUI calls setInputCommands() with current input state
 * 2. Engine calls updateState() during simulation update
 * 3. Agent translates boolean commands into velocity/acceleration
 *
 * Thread safety: Not thread-safe
 */
class InputControlAgent : public BaseAgent
{
public:
  /**
   * @brief Construct an input control agent
   * @param maxSpeed Maximum linear speed in m/s (default: 10.0)
   * @param maxAngularSpeed Maximum angular speed in rad/s (default: 1.0)
   */
  explicit InputControlAgent(double maxSpeed = 10.0,
                             double maxAngularSpeed = 1.0);

  /**
   * @brief Update state based on current input commands
   * @param currentState Current kinematic state
   * @return New desired kinematic state
   *
   * Translates boolean commands into velocity updates. Commands are directional
   * (agent converts to velocity in object's local frame).
   */
  [[nodiscard]] InertialState updateState(const InertialState& currentState) const;

  /**
   * @brief Set current input commands
   * @param commands Input commands from GUI
   *
   * Called from Engine when GUI updates input. Commands are stored and
   * applied during the next updateState() call.
   */
  void setInputCommands(const InputCommands& commands)
  {
    inputCommands_ = commands;
  }

  /**
   * @brief Get current input commands
   * @return Const reference to input commands
   */
  [[nodiscard]] const InputCommands& getInputCommands() const
  {
    return inputCommands_;
  }

  /**
   * @brief Set maximum linear speed
   * @param speed Maximum speed in m/s
   */
  void setMaxSpeed(double speed)
  {
    maxSpeed_ = speed;
  }

  /**
   * @brief Set maximum angular speed
   * @param speed Maximum angular speed in rad/s
   */
  void setMaxAngularSpeed(double speed)
  {
    maxAngularSpeed_ = speed;
  }

  /**
   * @brief Get maximum linear speed
   * @return Maximum speed in m/s
   */
  [[nodiscard]] double getMaxSpeed() const
  {
    return maxSpeed_;
  }

  /**
   * @brief Get maximum angular speed
   * @return Maximum angular speed in rad/s
   */
  [[nodiscard]] double getMaxAngularSpeed() const
  {
    return maxAngularSpeed_;
  }

private:
  InputCommands inputCommands_;
  double maxSpeed_{10.0};        // m/s
  double maxAngularSpeed_{1.0};  // rad/s
};

}  // namespace msd_sim

#endif  // INPUT_CONTROL_AGENT_HPP
