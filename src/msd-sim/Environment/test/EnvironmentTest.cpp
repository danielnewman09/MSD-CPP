
#include "msd-sim/Environment/src/Platform.hpp"
#include "msd-sim/Environment/src/WorldModel.hpp"
#include <gtest/gtest.h>

TEST(EnvTestSuite, WorldModelInitialization)
{
  msd_sim::WorldModel wm;
}

TEST(EnvTestSuite, PlatformMovement)
{
  msd_sim::Platform p{1};
}