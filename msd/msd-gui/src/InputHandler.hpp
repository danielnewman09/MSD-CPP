// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#ifndef INPUT_HANDLER_HPP
#define INPUT_HANDLER_HPP

#include <chrono>
#include <functional>
#include <vector>

#include <SDL3/SDL.h>

#include "msd-gui/src/InputState.hpp"

namespace msd_gui
{

/**
 * @brief Input mode defining how bindings should trigger
 *
 * - Continuous: Trigger every frame while key is held
 * - TriggerOnce: Trigger only on initial press (ignore hold)
 * - Interval: Trigger at fixed intervals while key is held
 * - PressAndHold: Trigger on release with hold duration percentage
 */
enum class InputMode : uint8_t
{
  Continuous,   // Trigger every frame while key is held
  TriggerOnce,  // Trigger only on initial press (ignore hold)
  Interval,     // Trigger at fixed intervals while key is held
  PressAndHold  // Trigger on release with hold duration
};

/**
 * @brief Binds a key to an action with specified input mode
 *
 * Encapsulates the binding logic separate from state tracking. The
 * shouldTrigger() method implements mode-specific logic (continuous, once,
 * interval).
 *
 * Thread safety: Not thread-safe (assumes single-threaded execution)
 */
class InputBinding
{
public:
  /**
   * @brief Construct an input binding
   * @param key SDL keycode to bind
   * @param mode Input mode (Continuous, TriggerOnce, Interval, PressAndHold)
   * @param action Function to execute when triggered
   * @param interval Interval duration for Interval mode (default: 0ms)
   */
  InputBinding(SDL_Keycode key,
               InputMode mode,
               std::function<void()> action,
               std::chrono::milliseconds interval = std::chrono::milliseconds{
                 0});

  /**
   * @brief Get the bound key
   * @return SDL keycode
   */
  [[nodiscard]] SDL_Keycode getKey() const
  {
    return key_;
  }

  /**
   * @brief Get the input mode
   * @return Input mode enum
   */
  [[nodiscard]] InputMode getMode() const
  {
    return mode_;
  }

  /**
   * @brief Check if binding should trigger given current input state
   * @param state Current input state
   * @return True if binding should execute
   *
   * Implements mode-specific trigger logic:
   * - Continuous: Returns true every frame while key is held
   * - TriggerOnce: Returns true only on justPressed
   * - Interval: Returns true at fixed intervals while key is held
   * - PressAndHold: Returns true on key release
   */
  bool shouldTrigger(const InputState& state);

  /**
   * @brief Execute the bound action
   *
   * Note: No exceptions are caught; action execution failure propagates.
   */
  void execute();

private:
  SDL_Keycode key_;
  InputMode mode_;
  std::chrono::milliseconds intervalMs_{0};
  std::function<void()> action_;
  std::chrono::milliseconds lastExecuteTime_{0};
};

/**
 * @brief Manages collection of input bindings and processes them based on
 * InputState
 *
 * Central coordination point for input processing. Owns InputState (single
 * source of truth) and evaluates all bindings to execute triggered actions.
 *
 * Thread safety: Not thread-safe
 */
class InputHandler
{
public:
  InputHandler() = default;
  ~InputHandler() = default;

  // Delete copy (owns unique state)
  InputHandler(const InputHandler&) = delete;
  InputHandler& operator=(const InputHandler&) = delete;

  // Allow move
  InputHandler(InputHandler&&) noexcept = default;
  InputHandler& operator=(InputHandler&&) noexcept = default;

  /**
   * @brief Add an input binding
   * @param binding InputBinding to add to the collection
   */
  void addBinding(InputBinding binding);

  /**
   * @brief Remove binding for a specific key
   * @param key SDL keycode to remove
   *
   * Removes the first binding matching the key. If multiple bindings exist
   * for the same key, only the first is removed.
   */
  void removeBinding(SDL_Keycode key);

  /**
   * @brief Clear all bindings
   */
  void clearBindings();

  /**
   * @brief Handle SDL event and update input state
   * @param event SDL event to process
   *
   * Processes SDL_EVENT_KEY_DOWN and SDL_EVENT_KEY_UP events to update
   * the internal InputState. Other event types are ignored.
   */
  void handleSDLEvent(const SDL_Event& event);

  /**
   * @brief Update frame timing and clear frame-specific flags
   * @param deltaTime Time elapsed since last update
   *
   * Advances the internal InputState time and clears justPressed flags.
   * Call this once per frame after processing all SDL events.
   */
  void update(std::chrono::milliseconds deltaTime);

  /**
   * @brief Process all bindings and execute triggered actions
   *
   * Evaluates all bindings against the current InputState and executes
   * actions for bindings that should trigger. Call this after update().
   */
  void processInput();

  /**
   * @brief Get read-only access to the input state
   * @return Const reference to InputState
   */
  [[nodiscard]] const InputState& getInputState() const
  {
    return inputState_;
  }

private:
  InputState inputState_;
  std::vector<InputBinding> bindings_;
};

}  // namespace msd_gui

#endif  // INPUT_HANDLER_HPP
