#ifndef EULER_ANGLES_HPP
#define EULER_ANGLES_HPP

#include <Eigen/Dense>
#include "msd-sim/src/Environment/Angle.hpp"

namespace msd_sim
{

struct EulerAngles
{
  Angle pitch;
  Angle roll;
  Angle yaw;
};


}  // namespace msd_sim

#endif  // EULER_ANGLES_HPP
