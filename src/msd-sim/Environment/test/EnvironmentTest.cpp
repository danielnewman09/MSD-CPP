
#include "Environment/src/Platform.hpp"
#include "Environment/src/WorldModel.hpp"
#include <gtest/gtest.h>

TEST(EnvTestSuite, WorldModelInitialization)
{
  msd_sim::WorldModel wm;
}

TEST(EnvTestSuite, PlatformMovement)
{
  msd_sim::Platform p{1};
}