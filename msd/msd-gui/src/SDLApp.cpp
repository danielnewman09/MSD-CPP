// Ticket: 0004_gui_framerate
// Ticket: 0005_camera_controller_sim
// Design: docs/designs/input-state-management/design.md
// Design: docs/designs/0005_camera_controller_sim/design.md
// Previous tickets: 0002_remove_rotation_from_gpu, 0001_link-gui-sim-object

#include "msd-gui/src/SDLApp.hpp"
#include "msd-assets/src/GeometryFactory.hpp"
#include "msd-gui/src/SDLUtils.hpp"

#include <cstdlib>
#include <format>
#include <iostream>
#include <random>
#include <vector>

namespace msd_gui
{

SDLApplication::SDLApplication(const std::string& dbPath)
  : engine_{dbPath},
    status_{Status::Starting},
    basePath_{SDL_GetBasePath() ? SDL_GetBasePath() : "/"}
{
  SDL_Log("SDLApplication: Starting initialization");
  SDL_Log("SDLApplication: dbPath = %s", dbPath.c_str());
  SDL_Log("SDLApplication: basePath = %s", basePath_.c_str());

  window_.reset(
    SDL_CreateWindow("MSD Application", 800, 600, SDL_WINDOW_RESIZABLE));

  if (!window_.get())
  {
    SDL_Log("ERROR: Failed to create SDL window: %s", SDL_GetError());
    throw SDLException("Failed to create SDL window");
  }
  SDL_Log("SDLApplication: Window created successfully");

  SDL_Log("SDLApplication: Creating player platform...");
  // Create player platform with visual object
  // The camera will reference the visual object's ReferenceFrame
  playerPlatformId_ =
    engine_.spawnPlayerPlatform("cube",
                                msd_sim::Coordinate{0., -15., -5.},
                                msd_sim::AngularCoordinate{0, M_PI / 2, 0});
  SDL_Log("SDLApplication: Player platform created with ID: %u",
          *playerPlatformId_);

  // Get the player platform's visual object's ReferenceFrame for the camera
  auto& worldModel = engine_.getWorldModel();
  msd_sim::ReferenceFrame* cameraFrame = nullptr;

  for (auto& platform : worldModel.getPlatforms())
  {
    if (platform.getId() == *playerPlatformId_)
    {
      cameraFrame = &platform.getInertialAsset().getReferenceFrame();
      break;
    }
  }

  if (!cameraFrame)
  {
    SDL_Log("ERROR: Failed to get camera reference frame from player platform");
    throw std::runtime_error("Failed to get camera reference frame");
  }

  SDL_Log("SDLApplication: Creating GPUManager with camera frame...");
  gpuManager_ = std::make_unique<GPUManager<AppShaderPolicy>>(
    *window_, *cameraFrame, basePath_);
  SDL_Log("SDLApplication: GPUManager created successfully");


  // Initialize input system
  inputHandler_ = std::make_unique<InputHandler>();

  // Setup input bindings
  setupInputBindings();

  // Initialize frame timing
  lastFrameTime_ = std::chrono::milliseconds{SDL_GetTicks()};

  status_ = Status::Running;
}

void SDLApplication::registerAssets()
{
  const auto& assetCache = engine_.getAssetRegistry().getAssetCache();

  for (const auto& asset : assetCache)
  {
    if (asset.hasVisualGeometry())
    {
      gpuManager_->registerGeometry(
        asset.getId(), asset.getVisualGeometry()->get().getVertices());
    }
  }
}

int SDLApplication::runApp()
{
  SDL_ShowWindow(window_.get());

  // Register assets with GPU manager before rendering
  registerAssets();

  std::chrono::milliseconds currentTime{0};
  std::chrono::milliseconds frameDelta{10};

  // Native: blocking main loop
  while (status_ == Status::Running)
  {
    // Calculate frame delta time
    currentTime += frameDelta;
    lastFrameTime_ = currentTime;

    handleEvents();
    gpuManager_->update(engine_);
    engine_.update(currentTime);
  }

  return EXIT_SUCCESS;
}

SDLApplication::Status SDLApplication::getStatus() const
{
  return status_;
}

void SDLApplication::setupInputBindings()
{
  // Z key: Spawn pyramid (TriggerOnce mode)
  inputHandler_->addBinding(InputBinding{SDLK_Z,
                                         InputMode::TriggerOnce,
                                         [this]()
                                         { spawnRandomObject("pyramid"); }});

  // V key: Spawn cube (TriggerOnce mode)
  inputHandler_->addBinding(InputBinding{
    SDLK_V, InputMode::TriggerOnce, [this]() { spawnRandomObject("cube"); }});

  // C key: Clear all objects (TriggerOnce mode)
  inputHandler_->addBinding(
    InputBinding{SDLK_C,
                 InputMode::TriggerOnce,
                 [this]()
                 {
                   engine_.getWorldModel().clearObjects();
                   SDL_Log("Cleared all objects");
                 }});
}

void SDLApplication::handleEvents()
{
  SDL_Event event;
  while (SDL_PollEvent(&event))
  {
    if (event.type == SDL_EVENT_QUIT)
    {
      status_ = Status::Exiting;
    }
    inputHandler_->handleSDLEvent(event);
  }

  // Process bindings BEFORE update (update clears justPressed flags)
  inputHandler_->processInput();

  // Update input state (advances time, clears justPressed flags for next frame)
  inputHandler_->update(frameDeltaTime_);

  // Update player input (forwards input to player platform's MotionController)
  updatePlayerInput();
}

void SDLApplication::updatePlayerInput()
{
  const auto& inputState = inputHandler_->getInputState();

  // Convert InputState to InputCommands
  msd_sim::InputCommands commands;
  commands.moveForward = inputState.isKeyPressed(SDLK_W);
  commands.moveBackward = inputState.isKeyPressed(SDLK_S);
  commands.moveLeft = inputState.isKeyPressed(SDLK_A);
  commands.moveRight = inputState.isKeyPressed(SDLK_D);
  commands.moveUp = inputState.isKeyPressed(SDLK_Q);
  commands.moveDown = inputState.isKeyPressed(SDLK_E);

  commands.pitchUp = inputState.isKeyPressed(SDLK_UP);
  commands.pitchDown = inputState.isKeyPressed(SDLK_DOWN);
  commands.yawLeft = inputState.isKeyPressed(SDLK_LEFT);
  commands.yawRight = inputState.isKeyPressed(SDLK_RIGHT);

  // Send to engine
  engine_.setPlayerInputCommands(commands);
}

void SDLApplication::spawnRandomObject(const std::string& geometryType)
{
  // Use random device for better randomness than rand()
  static std::random_device rd;
  static std::mt19937 gen{rd()};
  static std::uniform_real_distribution<double> posDist{-5.0, 5.0};
  static std::uniform_real_distribution<double> angleDist{-3.14159, 3.14159};
  static std::uniform_real_distribution<float> colorDist{0.0f, 1.0f};

  // Create random transform
  // msd_sim::Coordinate randomPos{posDist(gen), posDist(gen), posDist(gen)};

  msd_sim::Coordinate randomPos{0., 0., 0.};

  // msd_sim::AngularCoordinate randomOrientation{
  //   angleDist(gen),  // pitch (radians)
  //   angleDist(gen),  // roll (radians)
  //   angleDist(gen)   // yaw (radians)
  // };

  msd_sim::AngularCoordinate randomOrientation{0.1, 0., 0.};

  // Random color
  float r = colorDist(gen);
  float g = colorDist(gen);
  float b = colorDist(gen);

  auto object =
    engine_.spawnInertialObject(geometryType, randomPos, randomOrientation);
  gpuManager_->addObject(object, r, g, b);

  SDL_Log("Spawned %s at (%.2f, %.2f, %.2f) with orientation (%.2f, %.2f, "
          "%.2f) and color (%.2f, %.2f, %.2f). Total objects: %zu",
          geometryType.c_str(),
          randomPos.x(),
          randomPos.y(),
          randomPos.z(),
          randomOrientation.pitchDeg(),
          randomOrientation.rollDeg(),
          randomOrientation.yawDeg(),
          static_cast<double>(r),
          static_cast<double>(g),
          static_cast<double>(b),
          engine_.getWorldModel().getInertialAssets().size());
}


}  // namespace msd_gui
