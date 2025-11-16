#include <chrono>
#include "msd-sim/src/Engine.hpp"
#include "msd-sim/src/Environment/Platform.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"

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
