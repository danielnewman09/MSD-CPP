#include <gtest/gtest.h>
#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/Platform.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-sim/src/Physics/RigidBody/ConvexHull.hpp"

namespace
{
// Helper to create a minimal valid ConvexHull for Platform construction
msd_sim::ConvexHull createTestHull()
{
  std::vector<msd_sim::Coordinate> points = {
    msd_sim::Coordinate{0.0, 0.0, 0.0},
    msd_sim::Coordinate{1.0, 0.0, 0.0},
    msd_sim::Coordinate{0.5, 1.0, 0.0},
    msd_sim::Coordinate{0.5, 0.5, 1.0}};
  return msd_sim::ConvexHull{points};
}
}  // namespace

TEST(EnvTestSuite, WorldModelInitialization)
{
  msd_sim::WorldModel wm;
}

TEST(EnvTestSuite, PlatformMovement)
{
  msd_sim::ConvexHull hull = createTestHull();
  msd_sim::ReferenceFrame frame;
  msd_sim::Platform p{1, 0, 0, hull, 1.0, frame};
}