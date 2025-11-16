#ifndef PLATFORMSTATE_HPP
#define PLATFORMSTATE_HPP

#include <cstdint>

namespace msd_sim
{

struct PlatformState
{
  double positionX;
  double positionY;
  double velocityX;
  double velocityY;
  double rotationAngle;
};

} // namespace msd_sim

#endif