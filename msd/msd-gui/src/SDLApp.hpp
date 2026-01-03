// Ticket: 0002_remove_rotation_from_gpu
// Design: docs/designs/modularize-gpu-shader-system/design.md
// Previous ticket: 0001_link-gui-sim-object

#ifndef SDL_APP_HPP
#define SDL_APP_HPP

#include <memory>
#include <string>
#include <vector>

#include <SDL3/SDL.h>

#include "msd-assets/src/Asset.hpp"
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

private:
  struct SDLWindowDeleter
  {
    void operator()(SDL_Window* w) const
    {
      SDL_DestroyWindow(w);
    }
  };

  void handleEvents();
  void spawnRandomObject(const std::string& geometryType);

  msd_sim::Engine engine_;
  Status status_;
  std::string basePath_;

  std::unique_ptr<SDL_Window, SDLWindowDeleter> window_;

  // Use PositionOnlyShaderPolicy as default per design decision
  using AppGPUManager = GPUManager<PositionOnlyShaderPolicy>;
  std::unique_ptr<AppGPUManager> gpuManager_;

  // Mock object storage for demonstration
  std::vector<msd_sim::Object> mockObjects_;
  std::vector<msd_assets::Asset> mockAssets_;  // Own assets for Object references

  const float moveSpeed_{0.1f};
  const msd_sim::Coordinate unitX_{moveSpeed_, 0, 0};
  const msd_sim::Coordinate unitY_{0, moveSpeed_, 0};
  const msd_sim::Coordinate unitZ_{0, 0, moveSpeed_};
  const msd_sim::Angle rotSpeed_{0.05};
};

}  // namespace msd_gui

#endif  // SDL_APP_HPP