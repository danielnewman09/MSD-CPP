// Ticket: 0002_remove_rotation_from_gpu
// Design: docs/designs/modularize-gpu-shader-system/design.md
// Previous ticket: 0001_link-gui-sim-object

#include "msd-gui/src/SDLApp.hpp"
#include "msd-gui/src/SDLUtils.hpp"
#include "msd-assets/src/GeometryFactory.hpp"

#include <cstdlib>
#include <format>
#include <iostream>
#include <random>
#include <vector>

namespace msd_gui
{

SDLApplication::SDLApplication(const std::string& dbPath)
  : engine_{dbPath}, status_{Status::Starting}, basePath_{SDL_GetBasePath()}
{
  window_.reset(
    SDL_CreateWindow("MSD Application", 800, 600, SDL_WINDOW_RESIZABLE));

  if (!window_.get())
  {
    throw SDLException("Failed to create SDL window");
  }

  gpuManager_ = std::make_unique<AppGPUManager>(*window_, basePath_);

  // Get assets from the registry (they should be loaded by the engine from the database)
  auto& registry = engine_.getAssetRegistry();

  auto pyramidAsset = registry.getAsset("pyramid");
  if (pyramidAsset)
  {
    mockAssets_.push_back(pyramidAsset->get());
  }
  else
  {
    SDL_Log("WARNING: Pyramid asset not found in registry");
  }

  auto cubeAsset = registry.getAsset("cube");
  if (cubeAsset)
  {
    mockAssets_.push_back(cubeAsset->get());
  }
  else
  {
    SDL_Log("WARNING: Cube asset not found in registry");
  }

  status_ = Status::Running;
}


int SDLApplication::runApp()
{
  SDL_ShowWindow(window_.get());

  while (status_ == Status::Running)
  {
    handleEvents();
    gpuManager_->updateObjects(mockObjects_);
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
            // Spawn a random pyramid
            spawnRandomObject("pyramid");
            break;
          case SDLK_V:
            // Spawn a random cube
            spawnRandomObject("cube");
            break;
          case SDLK_X:
            // Remove the last object (if any exist)
            if (!mockObjects_.empty())
            {
              mockObjects_.pop_back();
              SDL_Log("Removed last object. Remaining objects: %zu",
                      mockObjects_.size());
            }
            break;
          case SDLK_C:
            // Clear all objects
            mockObjects_.clear();
            SDL_Log("Cleared all objects");
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

void SDLApplication::spawnRandomObject(const std::string& geometryType)
{
  // Use random device for better randomness than rand()
  static std::random_device rd;
  static std::mt19937 gen{rd()};
  static std::uniform_real_distribution<float> posDist{-5.0f, 5.0f};
  static std::uniform_real_distribution<float> angleDist{-3.14159f, 3.14159f};
  static std::uniform_real_distribution<float> colorDist{0.0f, 1.0f};

  // Find the asset
  const msd_assets::Asset* asset = nullptr;
  for (const auto& a : mockAssets_)
  {
    if (a.getName() == geometryType)
    {
      asset = &a;
      break;
    }
  }

  if (!asset)
  {
    SDL_Log("ERROR: Asset '%s' not found in mockAssets_", geometryType.c_str());
    return;
  }

  // Create random transform
  msd_sim::Coordinate randomPos{
    posDist(gen), posDist(gen), posDist(gen)};

  msd_sim::EulerAngles randomOrientation{
    msd_sim::Angle::fromRadians(angleDist(gen)),  // pitch
    msd_sim::Angle::fromRadians(angleDist(gen)),  // roll
    msd_sim::Angle::fromRadians(angleDist(gen))   // yaw
  };

  msd_sim::ReferenceFrame randomFrame{randomPos, randomOrientation};

  // Random color
  float r = colorDist(gen);
  float g = colorDist(gen);
  float b = colorDist(gen);

  // Create graphical object
  mockObjects_.push_back(
    msd_sim::Object::createGraphical(*asset, randomFrame, r, g, b));

  SDL_Log("Spawned %s at (%.2f, %.2f, %.2f) with orientation (%.2f, %.2f, "
          "%.2f) and color (%.2f, %.2f, %.2f). Total objects: %zu",
          geometryType.c_str(),
          randomPos.x(),
          randomPos.y(),
          randomPos.z(),
          randomOrientation.pitch.toDeg(),
          randomOrientation.roll.toDeg(),
          randomOrientation.yaw.toDeg(),
          r,
          g,
          b,
          mockObjects_.size());
}


}  // namespace msd_gui
