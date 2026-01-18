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
#include "msd-sim/src/Environment/MotionController.hpp"
#include "msd-sim/src/Physics/RigidBody/AssetInertial.hpp"

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
  explicit Platform(uint32_t platformId,
                    uint32_t assetInstanceId,
                    uint32_t assetId,
                    ConvexHull& hull,
                    double mass,
                    const ReferenceFrame& frame);

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
  void setAgent(std::unique_ptr<BaseAgent> agent)
  {
    agent_ = std::move(agent);
  }

  /**
   * @brief Get mutable agent pointer
   * @return Pointer to agent (nullptr if no agent)
   */
  BaseAgent* getAgent()
  {
    return agent_.get();
  }

  /**
   * @brief Get const agent pointer
   * @return Pointer to agent (nullptr if no agent)
   */
  const BaseAgent* getAgent() const
  {
    return agent_.get();
  }

  /**
   * @brief Check if platform has an agent
   * @return True if agent is present
   */
  bool hasAgent() const
  {
    return agent_ != nullptr;
  }

  // ========== Visual Object Linking ==========

  /**
   * @brief Get const visual object reference
   * @return Const reference to linked object
   * @throws std::bad_optional_access if no visual object linked
   */
  const AssetInertial& getInertialAsset() const
  {
    return inertialAsset_;
  }

  AssetInertial& getInertialAsset()
  {
    return inertialAsset_;
  }


  // ========== State Access ==========

  /**
   * @brief Get const state reference
   * @return Const reference to inertial state
   */
  const InertialState& getState() const
  {
    return inertialAsset_.getInertialState();
  }

  /**
   * @brief Get mutable state reference
   * @return Mutable reference to inertial state
   */
  InertialState& getState()
  {
    return inertialAsset_.getInertialState();
  }

  /**
   * @brief Get platform ID
   * @return Unique platform identifier
   */
  uint32_t getId() const
  {
    return id_;
  }

  // ========== Motion Control ==========

  /**
   * @brief Get mutable motion controller reference
   * @return Reference to motion controller
   *
   * Provides access to motion controller for updating transform based on input.
   * Typically used by Engine to forward player input commands.
   */
  MotionController& getMotionController()
  {
    return motionController_;
  }

  /**
   * @brief Get const motion controller reference
   * @return Const reference to motion controller
   */
  const MotionController& getMotionController() const
  {
    return motionController_;
  }

private:
  //! Platform ID
  uint32_t id_;

  //! Agent controlling the platform
  std::unique_ptr<BaseAgent> agent_;

  //! Optional reference to visual object in WorldModel
  AssetInertial inertialAsset_;

  //! Last update time
  std::chrono::milliseconds lastUpdateTime_;

  //! Motion controller for transform updates
  MotionController motionController_;
};

}  // namespace msd_sim

#endif  // PLATFORM_HPP