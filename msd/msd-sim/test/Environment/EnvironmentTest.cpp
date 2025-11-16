
#include "msd-sim/src/Environment/Platform.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include <gtest/gtest.h>

TEST(EnvTestSuite, WorldModelInitialization)
{
  msd_sim::WorldModel wm;
}

TEST(EnvTestSuite, PlatformMovement)
{
  msd_sim::Platform p{1};
}