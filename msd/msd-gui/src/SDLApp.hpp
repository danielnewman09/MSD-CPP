#ifndef SDL_APP_HPP
#define SDL_APP_HPP

#include <memory>
#include <stdexcept>
#include <string>

#include <SDL3/SDL.h>

#include "msd-gui/src/SDLGPUManager.hpp"

namespace msd_gui
{

class SDLException final : public std::runtime_error
{
public:
  explicit SDLException(const std::string& message)
    : std::runtime_error(message + ": " + SDL_GetError())
  {
  }
};

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

  void render();

  Status status_;
  std::string basePath_;

  std::unique_ptr<SDL_Window, SDLWindowDeleter> window_;
  std::unique_ptr<GPUManager> gpuManager_;
};

}  // namespace msd_gui

#endif  // SDL_APP_HPP