#include <chrono>
#include "msd-gui/src/SDLApp.hpp"
#include "msd-sim/src/Engine.hpp"
#include "msd-sim/src/Environment/Platform.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"
#include "msd-utils/src/PathUtils.hpp"

int main()
{
  std::string dbPath{"example_assets.db"};
  auto absolutePath = msd_utils::absolutePath(dbPath);

  msd_gui::SDLApplication application{absolutePath.string()};

  application.runApp();
  return 0;
}
