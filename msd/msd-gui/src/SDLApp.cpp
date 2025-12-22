#include "msd-gui/src/SDLApp.hpp"
#include "msd-gui/src/SDLUtils.hpp"

#include <cstdlib>
#include <format>
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
    auto& camera = gpuManager_->getCamera();
    auto& cameraFrame = camera.getReferenceFrame();
    auto newOrigin = cameraFrame.getOrigin();
    switch (event.type)
    {
      case SDL_EVENT_QUIT:
        status_ = Status::Exiting;
        break;
      case SDL_EVENT_KEY_DOWN:
        switch (event.key.key)
        {
          case SDLK_W:
            // Move forward in camera's local Z direction
            newOrigin -= cameraFrame.localToGlobalRelative(unitZ_);
            break;
          case SDLK_S:
            // Move backward in camera's local Z direction
            newOrigin += cameraFrame.localToGlobalRelative(unitZ_);
            break;
          case SDLK_A:
            // Move left in camera's local X direction (negative X)
            newOrigin -= cameraFrame.localToGlobalRelative(unitX_);
            break;
          case SDLK_D:
            // Move right in camera's local X direction (positive X)
            newOrigin += cameraFrame.localToGlobalRelative(unitX_);
            break;
          case SDLK_UP:
            // Pitch up (look up)
            cameraFrame.getEulerAngles().pitch += rotSpeed_;
            break;
          case SDLK_DOWN:
            // Pitch down (look down)
            cameraFrame.getEulerAngles().pitch -= rotSpeed_;
            break;
          case SDLK_LEFT:
            // Yaw left (turn left)
            cameraFrame.getEulerAngles().yaw += rotSpeed_;
            break;
          case SDLK_RIGHT:
            // Yaw right (turn right)
            cameraFrame.getEulerAngles().yaw -= rotSpeed_;
            break;
          case SDLK_Q:
            // Move up in camera's local Y direction
            newOrigin += cameraFrame.localToGlobalRelative(unitY_);
            break;
          case SDLK_E:
            // Move down in camera's local Y direction
            newOrigin -= cameraFrame.localToGlobalRelative(unitY_);
            break;
          case SDLK_Z:
            // Add a new random pyramid instance
            gpuManager_->addInstance(
              static_cast<float>(rand() % 10 - 5),    // Random X: -5 to 5
              static_cast<float>(rand() % 10 - 5),    // Random Y: -5 to 5
              static_cast<float>(rand() % 10 - 5),    // Random Z: -5 to 5
              static_cast<float>(rand()) / RAND_MAX,  // Random R
              static_cast<float>(rand()) / RAND_MAX,  // Random G
              static_cast<float>(rand()) / RAND_MAX   // Random B
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

    cameraFrame.setOrigin(newOrigin);

    // Log camera position using std::format
    auto logMsg = std::format("Camera at {:.2f}", cameraFrame.getOrigin());
    SDL_Log("%s", logMsg.c_str());
  }
}


}  // namespace msd_gui
