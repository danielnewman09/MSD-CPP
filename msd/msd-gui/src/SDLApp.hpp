#ifndef SDL_APP_HPP
#define SDL_APP_HPP

#include <memory>
#include <string>

#include <SDL3/SDL.h>

#include "msd-gui/src/SDLGPUManager.hpp"

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

  // Get the singleton instance
  static SDLApplication& getInstance();

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

  // Private constructor
  SDLApplication();

  // Private destructor
  ~SDLApplication() = default;

  void handleEvents();

  Status status_;
  std::string basePath_;

  std::unique_ptr<SDL_Window, SDLWindowDeleter> window_;
  std::unique_ptr<GPUManager> gpuManager_;

  const float moveSpeed_{0.1f};
  const msd_sim::Coordinate unitX_{moveSpeed_, 0, 0};
  const msd_sim::Coordinate unitY_{0, moveSpeed_, 0};
  const msd_sim::Coordinate unitZ_{0, 0, moveSpeed_};
  const msd_sim::Angle rotSpeed_{0.05};
};

}  // namespace msd_gui

#endif  // SDL_APP_HPP