#include "msd-gui/src/SDLApp.hpp"

#include <iostream>
#include <vector>

namespace msd_gui
{

SDLApplication& SDLApplication::getInstance()
{
  static SDLApplication instance;
  return instance;
}

SDLApplication::SDLApplication()
  : status_{Status::Starting}, basePath_{SDL_GetBasePath()}
{
  window_.reset(
    SDL_CreateWindow("MSD Application", 800, 600, SDL_WINDOW_RESIZABLE));

  if (!window_.get())
  {
    throw SDLException("Failed to create SDL window");
  }

  gpuManager_ = std::make_unique<GPUManager>(*window_, basePath_);


  status_ = Status::Running;
}


int SDLApplication::runApp()
{
  SDL_ShowWindow(window_.get());

  while (status_ == Status::Running)
  {
    handleEvents();
    gpuManager_->render();
  }

  return EXIT_SUCCESS;
}


SDLApplication::Status SDLApplication::getStatus() const
{
  return status_;
}

void SDLApplication::handleEvents()
{
  SDL_Event event;
  while (SDL_PollEvent(&event))
  {
    switch (event.type)
    {
      case SDL_EVENT_QUIT:
        status_ = Status::Exiting;
        break;
      default:
        break;
    }
  }
}


}  // namespace msd_gui
