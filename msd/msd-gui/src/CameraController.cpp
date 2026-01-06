// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#include "msd-gui/src/CameraController.hpp"

#include <SDL3/SDL.h>

namespace msd_gui
{

CameraController::CameraController(Camera3D& camera,
                                   float moveSpeed,
                                   msd_sim::Angle rotSpeed)
  : camera_{camera}, moveSpeed_{moveSpeed}, rotSpeed_{rotSpeed}
{
}

void CameraController::updateFromInput(const InputState& inputState,
                                       std::chrono::milliseconds deltaTime)
{
  auto& cameraFrame = camera_.getReferenceFrame();
  auto newOrigin = cameraFrame.getOrigin();
  auto& eulerAngles = cameraFrame.getEulerAngles();

  // Linear movement in camera's local frame
  // Note: Camera's local Z is forward (negative Z is into the screen in camera space)
  if (inputState.isKeyPressed(SDLK_W))
  {
    // Move forward in camera's local Z direction
    newOrigin -= cameraFrame.localToGlobalRelative(
      msd_sim::Coordinate{0, 0, moveSpeed_ * sensitivity_});
  }
  if (inputState.isKeyPressed(SDLK_S))
  {
    // Move backward in camera's local Z direction
    newOrigin += cameraFrame.localToGlobalRelative(
      msd_sim::Coordinate{0, 0, moveSpeed_ * sensitivity_});
  }
  if (inputState.isKeyPressed(SDLK_A))
  {
    // Move left in camera's local X direction
    newOrigin -= cameraFrame.localToGlobalRelative(
      msd_sim::Coordinate{moveSpeed_ * sensitivity_, 0, 0});
  }
  if (inputState.isKeyPressed(SDLK_D))
  {
    // Move right in camera's local X direction
    newOrigin += cameraFrame.localToGlobalRelative(
      msd_sim::Coordinate{moveSpeed_ * sensitivity_, 0, 0});
  }
  if (inputState.isKeyPressed(SDLK_Q))
  {
    // Move up in camera's local Y direction
    newOrigin += cameraFrame.localToGlobalRelative(
      msd_sim::Coordinate{0, moveSpeed_ * sensitivity_, 0});
  }
  if (inputState.isKeyPressed(SDLK_E))
  {
    // Move down in camera's local Y direction
    newOrigin -= cameraFrame.localToGlobalRelative(
      msd_sim::Coordinate{0, moveSpeed_ * sensitivity_, 0});
  }

  // Apply movement
  cameraFrame.setOrigin(newOrigin);

  // Rotation
  if (inputState.isKeyPressed(SDLK_UP))
  {
    // Pitch up (look up)
    eulerAngles.pitch += rotSpeed_;
  }
  if (inputState.isKeyPressed(SDLK_DOWN))
  {
    // Pitch down (look down)
    eulerAngles.pitch -= rotSpeed_;
  }
  if (inputState.isKeyPressed(SDLK_LEFT))
  {
    // Yaw left (turn left)
    eulerAngles.yaw += rotSpeed_;
  }
  if (inputState.isKeyPressed(SDLK_RIGHT))
  {
    // Yaw right (turn right)
    eulerAngles.yaw -= rotSpeed_;
  }

  // Update rotation matrix in reference frame
  cameraFrame.setRotation(eulerAngles);
}

}  // namespace msd_gui
