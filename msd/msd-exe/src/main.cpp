#include <string>
#include "msd-gui/src/SDLApp.hpp"
#include "msd-utils/src/PathUtils.hpp"

int main()
{
#ifdef __EMSCRIPTEN__
  // For Emscripten/WebAssembly, the database is embedded in the virtual
  // filesystem at /assets/example_assets.db via --preload-file
  std::string dbPath{"/assets/example_assets.db"};
#else
  // For native builds, resolve the path relative to the executable
  std::string dbPath{"example_assets.db"};
  auto absolutePath = msd_utils::absolutePath(dbPath);
  dbPath = absolutePath.string();
#endif

  msd_gui::SDLApplication application{dbPath};

  application.runApp();
  return 0;
}
