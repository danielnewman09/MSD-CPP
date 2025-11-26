#ifndef PLATFORM_HPP
#define PLATFORM_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include "msd-sim/src/Agent/BaseAgent.hpp"
#include "msd-sim/src/Environment/InertialState.hpp"
namespace msd_sim
{

class Platform
{
public:
  Platform(uint32_t id);
  ~Platform();

  /*!
   * \brief Update the platform
   * \param currTime the current simulation time to apply
   */
  void update(const std::chrono::milliseconds& currTime);

private:
  //! State of the platform
  InertialState state_;

  //! Agent controlling the platform
  std::unique_ptr<BaseAgent> agent_;

  //! Sensor attached to the platform
  // Sensor sensor_;

  //! Platform ID
  uint32_t id_;

  std::chrono::milliseconds lastUpdateTime_;
};

}  // namespace msd_sim

#endif  // PLATFORM_HPP