#include "msd-gui/src/SDLApp.hpp"
#include "msd-gui/src/SDLUtils.hpp"

#include <cstdlib>
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
      case SDL_EVENT_KEY_DOWN:
        switch (event.key.key)
        {
          case SDLK_W:
            cameraPosZ_ -= moveSpeed_;  // Move forward (toward negative Z)
            gpuManager_->setCameraTransform(cameraPosX_, cameraPosY_, cameraPosZ_, cameraRotX_, cameraRotY_, 0.0f);
            break;
          case SDLK_S:
            cameraPosZ_ += moveSpeed_;  // Move backward (toward positive Z)
            gpuManager_->setCameraTransform(cameraPosX_, cameraPosY_, cameraPosZ_, cameraRotX_, cameraRotY_, 0.0f);
            break;
          case SDLK_A:
            cameraPosX_ -= moveSpeed_;
            gpuManager_->setCameraTransform(cameraPosX_, cameraPosY_, cameraPosZ_, cameraRotX_, cameraRotY_, 0.0f);
            break;
          case SDLK_D:
            cameraPosX_ += moveSpeed_;
            gpuManager_->setCameraTransform(cameraPosX_, cameraPosY_, cameraPosZ_, cameraRotX_, cameraRotY_, 0.0f);
            break;
          case SDLK_UP:
            cameraRotX_ += rotSpeed_;
            gpuManager_->setCameraTransform(cameraPosX_, cameraPosY_, cameraPosZ_, cameraRotX_, cameraRotY_, 0.0f);
            break;
          case SDLK_DOWN:
            cameraRotX_ -= rotSpeed_;
            gpuManager_->setCameraTransform(cameraPosX_, cameraPosY_, cameraPosZ_, cameraRotX_, cameraRotY_, 0.0f);
            break;
          case SDLK_LEFT:
            cameraRotY_ -= rotSpeed_;
            gpuManager_->setCameraTransform(cameraPosX_, cameraPosY_, cameraPosZ_, cameraRotX_, cameraRotY_, 0.0f);
            break;
          case SDLK_RIGHT:
            cameraRotY_ += rotSpeed_;
            gpuManager_->setCameraTransform(cameraPosX_, cameraPosY_, cameraPosZ_, cameraRotX_, cameraRotY_, 0.0f);
            break;
          case SDLK_Q:
            cameraPosY_ += moveSpeed_;
            gpuManager_->setCameraTransform(cameraPosX_, cameraPosY_, cameraPosZ_, cameraRotX_, cameraRotY_, 0.0f);
            break;
          case SDLK_E:
            cameraPosY_ -= moveSpeed_;
            gpuManager_->setCameraTransform(cameraPosX_, cameraPosY_, cameraPosZ_, cameraRotX_, cameraRotY_, 0.0f);
            break;
          case SDLK_Z:
            // Add a new random pyramid instance
            gpuManager_->addInstance(
              static_cast<float>(rand() % 10 - 5),  // Random X: -5 to 5
              static_cast<float>(rand() % 10 - 5),  // Random Y: -5 to 5
              static_cast<float>(rand() % 10 - 5),  // Random Z: -5 to 5
              static_cast<float>(rand()) / RAND_MAX, // Random R
              static_cast<float>(rand()) / RAND_MAX, // Random G
              static_cast<float>(rand()) / RAND_MAX  // Random B
            );
            break;
          case SDLK_X:
            // Remove the last instance (if any exist)
            if (gpuManager_->getInstanceCount() > 0)
            {
              gpuManager_->removeInstance(gpuManager_->getInstanceCount() - 1);
            }
            break;
          case SDLK_C:
            // Clear all instances
            gpuManager_->clearInstances();
            break;
          default:
            break;
        }
        break;
      default:
        break;
    }
  }
}


}  // namespace msd_gui
