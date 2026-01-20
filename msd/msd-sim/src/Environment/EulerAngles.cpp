// Ticket: 0023a_force_application_scaffolding
// Design: docs/designs/0023a_force_application_scaffolding/design.md

#include "msd-sim/src/Environment/EulerAngles.hpp"

namespace msd_sim
{

Coordinate EulerAngles::toCoordinate() const
{
  return Coordinate{pitch.getRad(), roll.getRad(), yaw.getRad()};
}

EulerAngles EulerAngles::fromCoordinate(const Coordinate& coord)
{
  return EulerAngles{
    Angle::fromRadians(coord.x()),
    Angle::fromRadians(coord.y()),
    Angle::fromRadians(coord.z())
  };
}

}  // namespace msd_sim
