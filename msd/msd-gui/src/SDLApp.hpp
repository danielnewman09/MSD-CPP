// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md
// Previous tickets: 0002_remove_rotation_from_gpu, 0001_link-gui-sim-object

#ifndef SDL_APP_HPP
#define SDL_APP_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <SDL3/SDL.h>

#include "msd-assets/src/Asset.hpp"
#include "msd-gui/src/CameraController.hpp"
#include "msd-gui/src/InputHandler.hpp"
#include "msd-gui/src/SDLGPUManager.hpp"
#include "msd-gui/src/ShaderPolicy.hpp"
#include "msd-sim/src/Engine.hpp"
#include "msd-sim/src/Environment/Object.hpp"

namespace msd_gui
{

class SDLApplication
{
public:
  enum class Status : uint8_t
  {
    Starting,
    Running,
    Paused,
    Error,
    Exiting
  };

  SDLApplication(const std::string& dbPath);

  // Delete copy constructor and assignment operator
  SDLApplication(const SDLApplication&) = delete;
  SDLApplication& operator=(const SDLApplication&) = delete;

  // Delete move constructor and assignment operator
  SDLApplication(SDLApplication&&) = delete;
  SDLApplication& operator=(SDLApplication&&) = delete;

  int runApp();

  Status getStatus() const;

#ifdef __EMSCRIPTEN__
  // Emscripten requires a callback-based main loop instead of blocking
  void runFrame();
  static void emscriptenMainLoop(void* arg);
#endif

private:
  struct SDLWindowDeleter
  {
    void operator()(SDL_Window* w) const
    {
      SDL_DestroyWindow(w);
    }
  };

  void handleEvents();
  void setupInputBindings();
  void updatePlayerInput();
  void spawnRandomObject(const std::string& geometryType);

  msd_sim::Engine engine_;
  Status status_;
  std::string basePath_;

  std::unique_ptr<SDL_Window, SDLWindowDeleter> window_;

  // Use FullTransformShaderPolicy per previous design decisions
  using AppGPUManager = GPUManager<FullTransformShaderPolicy>;
  std::unique_ptr<AppGPUManager> gpuManager_;

  // Input management
  std::unique_ptr<InputHandler> inputHandler_;
  std::unique_ptr<CameraController> cameraController_;

  // Frame timing for delta time
  std::chrono::milliseconds lastFrameTime_{0};
  std::chrono::milliseconds frameDeltaTime_{16};  // Default 60 FPS

  // Mock object storage for demonstration
  std::vector<msd_sim::Object> mockObjects_;
  std::vector<msd_assets::Asset>
    mockAssets_;  // Own assets for Object references
};

}  // namespace msd_gui

#endif  // SDL_APP_HPP