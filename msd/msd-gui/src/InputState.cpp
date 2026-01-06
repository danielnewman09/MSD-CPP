// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#include "msd-gui/src/InputState.hpp"

namespace msd_gui
{

void InputState::updateKey(SDL_Keycode key, bool pressed)
{
  auto& keyState = keyStates_[key];

  if (pressed && !keyState.pressed)
  {
    // Key just pressed
    keyState.pressed = true;
    keyState.justPressed = true;
    keyState.pressTime = currentTime_;
    keyState.lastTriggerTime = currentTime_;
  }
  else if (!pressed && keyState.pressed)
  {
    // Key just released
    keyState.pressed = false;
    keyState.justPressed = false;
  }
}

bool InputState::isKeyPressed(SDL_Keycode key) const
{
  auto it = keyStates_.find(key);
  if (it != keyStates_.end())
  {
    return it->second.pressed;
  }
  return false;
}

bool InputState::isKeyJustPressed(SDL_Keycode key) const
{
  auto it = keyStates_.find(key);
  if (it != keyStates_.end())
  {
    return it->second.justPressed;
  }
  return false;
}

bool InputState::isKeyHeld(SDL_Keycode key) const
{
  // Currently equivalent to isKeyPressed
  // Provided for semantic clarity in client code
  return isKeyPressed(key);
}

std::chrono::milliseconds InputState::getKeyHoldDuration(SDL_Keycode key) const
{
  auto it = keyStates_.find(key);
  if (it != keyStates_.end() && it->second.pressed)
  {
    return currentTime_ - it->second.pressTime;
  }
  return std::chrono::milliseconds{0};
}

void InputState::update(std::chrono::milliseconds deltaTime)
{
  currentTime_ += deltaTime;

  // Clear justPressed flags for all keys
  for (auto& [key, state] : keyStates_)
  {
    state.justPressed = false;
  }
}

void InputState::reset()
{
  keyStates_.clear();
  currentTime_ = std::chrono::milliseconds{0};
}

}  // namespace msd_gui
