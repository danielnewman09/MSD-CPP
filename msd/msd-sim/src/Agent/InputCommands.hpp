// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#ifndef INPUT_COMMANDS_HPP
#define INPUT_COMMANDS_HPP

namespace msd_sim
{

/**
 * @brief Plain data structure representing current input command state
 *
 * This struct is the bridge between msd-gui (input source) and msd-sim (consumer).
 * It decouples input representation from simulation logic.
 *
 * Boolean flags indicate command state; velocity/force calculation happens in the agent.
 * Extensible for future commands (attack, interact, etc.).
 *
 * Thread safety: Value type (safe to copy)
 */
struct InputCommands
{
  // Linear movement
  bool moveForward{false};
  bool moveBackward{false};
  bool moveLeft{false};
  bool moveRight{false};
  bool moveUp{false};
  bool moveDown{false};

  // Rotation
  bool pitchUp{false};
  bool pitchDown{false};
  bool yawLeft{false};
  bool yawRight{false};
  bool rollLeft{false};
  bool rollRight{false};

  // Discrete actions
  bool jump{false};

  /**
   * @brief Reset all commands to false
   */
  void reset()
  {
    moveForward = moveBackward = moveLeft = moveRight = false;
    moveUp = moveDown = false;
    pitchUp = pitchDown = yawLeft = yawRight = false;
    rollLeft = rollRight = false;
    jump = false;
  }
};

}  // namespace msd_sim

#endif  // INPUT_COMMANDS_HPP
