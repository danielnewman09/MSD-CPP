#include <chrono>
#include "msd-gui/src/SDLApp.hpp"
#include "msd-sim/src/Engine.hpp"
#include "msd-sim/src/Environment/Platform.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"

int main()
{
  auto& application = msd_gui::SDLApplication::getInstance();

  application.runApp();
  return 0;
}
