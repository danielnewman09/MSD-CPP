#include "msd-sim/Engine.hpp"
#include "msd-sim/Environment/src/Platform.hpp"
#include "msd-sim/Environment/src/WorldModel.hpp"
#include <chrono>

int main()
{
  msd_sim::Engine engine{};

  std::chrono::milliseconds simTime{0};
  std::chrono::milliseconds dt{10};

  while (true)
    {
      engine.update(simTime);

      simTime += dt;
    }
  return 0;
}
