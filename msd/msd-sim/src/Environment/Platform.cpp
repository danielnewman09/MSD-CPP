// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#include "msd-sim/src/Environment/Platform.hpp"
#include "msd-sim/src/Environment/Object.hpp"
#include <iostream>

namespace msd_sim
{

// Constructor
Platform::Platform(uint32_t id)
  : id_{id}, lastUpdateTime_{std::chrono::milliseconds{0}}
{
  std::cout << "Platform " << id_ << " created.\n";
}

// Destructor
Platform::~Platform()
{
  std::cout << "Platform " << id_ << " destroyed.\n";
}

// Update method
void Platform::update(const std::chrono::milliseconds& currTime)
{
  // Note: Delta time could be passed to agent for frame-rate independent updates
  // Currently agent->updateState() doesn't use delta time (future enhancement)

  // Update state via agent if present
  if (agent_)
  {
    state_ = agent_->updateState(state_);
  }

  // Sync visual object position and rotation if linked
  if (visualObject_.has_value())
  {
    Object& obj = visualObject_->get();
    obj.setPosition(state_.position);
    obj.getTransform().setRotation(state_.angularPosition);
  }

  // Update the last update time to current time
  lastUpdateTime_ = currTime;
}
}  // namespace msd_sim