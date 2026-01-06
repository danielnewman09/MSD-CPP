// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#include "msd-gui/src/InputHandler.hpp"

namespace msd_gui
{

//------------------------------------------------------------------------------
// InputBinding Implementation
//------------------------------------------------------------------------------

InputBinding::InputBinding(SDL_Keycode key,
                           InputMode mode,
                           std::function<void()> action,
                           std::chrono::milliseconds interval)
  : key_{key}, mode_{mode}, intervalMs_{interval}, action_{std::move(action)}
{
}

bool InputBinding::shouldTrigger(const InputState& state)
{
  switch (mode_)
  {
    case InputMode::Continuous:
      // Trigger every frame while key is held
      return state.isKeyPressed(key_);

    case InputMode::TriggerOnce:
      // Trigger only on initial press
      return state.isKeyJustPressed(key_);

    case InputMode::Interval:
    {
      // Trigger at fixed intervals while key is held
      if (!state.isKeyPressed(key_))
      {
        return false;
      }

      auto holdDuration = state.getKeyHoldDuration(key_);
      auto timeSinceLastTrigger = holdDuration - lastExecuteTime_;

      if (timeSinceLastTrigger >= intervalMs_)
      {
        lastExecuteTime_ = holdDuration;
        return true;
      }
      return false;
    }

    case InputMode::PressAndHold:
      // Trigger on key release
      // Note: Current InputState doesn't track "just released" flag
      // This mode is documented but not fully implemented
      // TODO: Add "justReleased" tracking to InputState for PressAndHold mode
      return false;

    default:
      return false;
  }
}

void InputBinding::execute()
{
  if (action_)
  {
    action_();
  }
}

//------------------------------------------------------------------------------
// InputHandler Implementation
//------------------------------------------------------------------------------

InputHandler::InputHandler() : inputState_{}, bindings_{}
{
}

void InputHandler::addBinding(InputBinding binding)
{
  bindings_.push_back(std::move(binding));
}

void InputHandler::removeBinding(SDL_Keycode key)
{
  // Remove first binding matching the key
  auto it = std::find_if(bindings_.begin(),
                         bindings_.end(),
                         [key](const InputBinding& binding)
                         { return binding.getKey() == key; });

  if (it != bindings_.end())
  {
    bindings_.erase(it);
  }
}

void InputHandler::clearBindings()
{
  bindings_.clear();
}

void InputHandler::handleSDLEvent(const SDL_Event& event)
{
  switch (event.type)
  {
    case SDL_EVENT_KEY_DOWN:
    {
      SDL_Keycode key = event.key.key;
      inputState_.updateKey(key, true);

      // Check TriggerOnce bindings immediately on key down
      // This ensures they fire before update() clears justPressed
      for (auto& binding : bindings_)
      {
        if (binding.getKey() == key && binding.getMode() == InputMode::TriggerOnce)
        {
          binding.execute();
        }
      }
      break;
    }

    case SDL_EVENT_KEY_UP:
      inputState_.updateKey(event.key.key, false);
      break;

    default:
      // Ignore other event types
      break;
  }
}

void InputHandler::update(std::chrono::milliseconds deltaTime)
{
  inputState_.update(deltaTime);
}

void InputHandler::processInput()
{
  for (auto& binding : bindings_)
  {
    // Skip TriggerOnce bindings - they're handled immediately in handleSDLEvent
    if (binding.getMode() == InputMode::TriggerOnce)
    {
      continue;
    }

    if (binding.shouldTrigger(inputState_))
    {
      binding.execute();
    }
  }
}

}  // namespace msd_gui
