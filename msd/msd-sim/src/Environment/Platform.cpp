// Ticket: 0004_gui_framerate
// Ticket: 0005_camera_controller_sim
// Design: docs/designs/input-state-management/design.md
// Design: docs/designs/0005_camera_controller_sim/design.md

#include "msd-sim/src/Environment/Platform.hpp"
#include <iostream>

namespace msd_sim
{

// Constructor
Platform::Platform(uint32_t platformId,
                   uint32_t assetInstanceId,
                   uint32_t assetId,
                   ConvexHull& hull,
                   double mass,
                   const ReferenceFrame& frame)
  : id_{platformId},
    inertialAsset_{assetId, assetInstanceId, hull, mass, frame},
    lastUpdateTime_{std::chrono::milliseconds{0}}
{
  std::cout << "Platform " << id_ << " created.\n";
}

// Update method
void Platform::update(const std::chrono::milliseconds& currTime)
{
  // Note: Delta time could be passed to agent for frame-rate independent
  // updates Currently agent->updateState() doesn't use delta time (future
  // enhancement)

  // Update the last update time to current time
  lastUpdateTime_ = currTime;
}
}  // namespace msd_sim