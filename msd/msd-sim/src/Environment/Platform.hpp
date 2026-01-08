// Ticket: 0004_gui_framerate
// Ticket: 0005_camera_controller_sim
// Design: docs/designs/input-state-management/design.md
// Design: docs/designs/0005_camera_controller_sim/design.md

#ifndef PLATFORM_HPP
#define PLATFORM_HPP

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include "msd-sim/src/Agent/BaseAgent.hpp"
#include "msd-sim/src/Environment/InertialState.hpp"
#include "msd-sim/src/Environment/MotionController.hpp"

namespace msd_sim
{

// Forward declaration to avoid circular dependency
class Object;

/**
 * @brief Platform entity with kinematic state and agent control
 *
 * Platform owns an agent that controls its state and optionally links to
 * an Object in WorldModel for visual representation synchronization.
 *
 * Per AC7: Agent logic is placed in Platform (not Object)
 * Per AC3: Each Platform has its own non-unique internal state
 *
 * Thread safety: Not thread-safe
 */
class Platform
{
public:
  explicit Platform(uint32_t id);
  ~Platform();

  // Delete copy (unique_ptr member)
  Platform(const Platform&) = delete;
  Platform& operator=(const Platform&) = delete;

  // Enable move
  Platform(Platform&&) noexcept = default;
  Platform& operator=(Platform&&) noexcept = default;

  /*!
   * \brief Update the platform state via agent and sync visual object
   * \param currTime the current simulation time to apply
   *
   * If an agent is present, calls agent->updateState() to compute new state.
   * If a visual object is linked, synchronizes object position and rotation.
   */
  void update(const std::chrono::milliseconds& currTime);

  // ========== Agent Management ==========

  /**
   * @brief Set the agent controlling this platform
   * @param agent Unique pointer to agent (transfers ownership)
   */
  void setAgent(std::unique_ptr<BaseAgent> agent) { agent_ = std::move(agent); }

  /**
   * @brief Get mutable agent pointer
   * @return Pointer to agent (nullptr if no agent)
   */
  BaseAgent* getAgent() { return agent_.get(); }

  /**
   * @brief Get const agent pointer
   * @return Pointer to agent (nullptr if no agent)
   */
  const BaseAgent* getAgent() const { return agent_.get(); }

  /**
   * @brief Check if platform has an agent
   * @return True if agent is present
   */
  bool hasAgent() const { return agent_ != nullptr; }

  // ========== Visual Object Linking ==========

  /**
   * @brief Link platform to a visual object in WorldModel
   * @param object Object reference (non-owning)
   *
   * When linked, platform synchronizes object position/rotation during update().
   */
  void setVisualObject(Object& object) { visualObject_ = object; }

  /**
   * @brief Check if platform has a linked visual object
   * @return True if visual object is linked
   */
  bool hasVisualObject() const { return visualObject_.has_value(); }

  /**
   * @brief Get mutable visual object reference
   * @return Reference to linked object
   * @throws std::bad_optional_access if no visual object linked
   */
  Object& getVisualObject() { return visualObject_->get(); }

  /**
   * @brief Get const visual object reference
   * @return Const reference to linked object
   * @throws std::bad_optional_access if no visual object linked
   */
  const Object& getVisualObject() const { return visualObject_->get(); }

  // ========== State Access ==========

  /**
   * @brief Get const state reference
   * @return Const reference to inertial state
   */
  const InertialState& getState() const { return state_; }

  /**
   * @brief Get mutable state reference
   * @return Mutable reference to inertial state
   */
  InertialState& getState() { return state_; }

  /**
   * @brief Get platform ID
   * @return Unique platform identifier
   */
  uint32_t getId() const { return id_; }

  // ========== Motion Control ==========

  /**
   * @brief Get mutable motion controller reference
   * @return Reference to motion controller
   *
   * Provides access to motion controller for updating transform based on input.
   * Typically used by Engine to forward player input commands.
   */
  MotionController& getMotionController() { return motionController_; }

  /**
   * @brief Get const motion controller reference
   * @return Const reference to motion controller
   */
  const MotionController& getMotionController() const { return motionController_; }

private:
  //! State of the platform
  InertialState state_;

  //! Agent controlling the platform
  std::unique_ptr<BaseAgent> agent_;

  //! Optional reference to visual object in WorldModel
  std::optional<std::reference_wrapper<Object>> visualObject_;

  //! Motion controller for transform updates
  MotionController motionController_;

  //! Sensor attached to the platform
  // Sensor sensor_;

  //! Platform ID
  uint32_t id_;

  std::chrono::milliseconds lastUpdateTime_;
};

}  // namespace msd_sim

#endif  // PLATFORM_HPP