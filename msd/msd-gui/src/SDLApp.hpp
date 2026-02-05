// Ticket: 0004_gui_framerate
// Ticket: 0005_camera_controller_sim
// Design: docs/designs/input-state-management/design.md
// Design: docs/designs/0005_camera_controller_sim/design.md
// Previous tickets: 0002_remove_rotation_from_gpu, 0001_link-gui-sim-object

#ifndef SDL_APP_HPP
#define SDL_APP_HPP

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <SDL3/SDL.h>

#include "msd-gui/src/InputHandler.hpp"
#include "msd-gui/src/SDLGPUManager.hpp"
#include "msd-gui/src/ShaderPolicy.hpp"
#include "msd-sim/src/Engine.hpp"

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

  explicit SDLApplication(const std::string& dbPath);

  // Delete copy constructor and assignment operator
  SDLApplication(const SDLApplication&) = delete;
  SDLApplication& operator=(const SDLApplication&) = delete;

  // Delete move constructor and assignment operator
  SDLApplication(SDLApplication&&) = delete;
  SDLApplication& operator=(SDLApplication&&) = delete;

  ~SDLApplication() = default;

  int runApp();

  Status getStatus() const;

private:
  struct SDLWindowDeleter
  {
    void operator()(SDL_Window* w) const
    {
      SDL_DestroyWindow(w);
    }
  };
  void registerAssets();
  void handleEvents();
  void setupInputBindings();
  void updatePlayerInput();
  void spawnRandomObject(const std::string& geometryType);

  msd_sim::Engine engine_;
  Status status_;
  std::string basePath_;

  std::unique_ptr<SDL_Window, SDLWindowDeleter> window_;

  // Use FullTransformShaderPolicy per previous design decisions
  using AppShaderPolicy = FullTransformShaderPolicy;
  std::unique_ptr<GPUManager<AppShaderPolicy>> gpuManager_;

  // Input management
  std::unique_ptr<InputHandler> inputHandler_;

  // Player platform ID for input forwarding
  std::optional<uint32_t> playerPlatformId_;

  // Frame timing for delta time
  std::chrono::milliseconds lastFrameTime_{0};
  std::chrono::milliseconds frameDeltaTime_{16};  // Default 60 FPS
};

}  // namespace msd_gui

#endif  // SDL_APP_HPP