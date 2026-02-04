// Ticket: 0004_gui_framerate
// Design: docs/designs/input-state-management/design.md

#ifndef INPUT_STATE_HPP
#define INPUT_STATE_HPP

#include <chrono>
#include <unordered_map>

#include <SDL3/SDL.h>

namespace msd_gui
{

/**
 * @brief State information for a single key
 *
 * Tracks whether a key is pressed, just pressed (this frame), and timing information
 * for duration-based queries and interval-based triggers.
 */
struct KeyState
{
  bool pressed{false};              // Currently pressed
  bool justPressed{false};          // Pressed this frame (cleared each update)
  std::chrono::milliseconds pressTime{0};        // When key was first pressed
  std::chrono::milliseconds lastTriggerTime{0};  // Last time action triggered (for interval mode)
};

/**
 * @brief Tracks the current state of all keyboard inputs
 *
 * Provides centralized input state tracking with timestamp information for
 * duration-based queries. Separates state tracking from input handling logic.
 *
 * Thread safety: Not thread-safe (single-threaded GUI operation assumed)
 */
class InputState
{
public:
  InputState() = default;
  ~InputState() = default;

  InputState(const InputState&) = default;
  InputState& operator=(const InputState&) = default;
  InputState(InputState&&) noexcept = default;
  InputState& operator=(InputState&&) noexcept = default;

  /**
   * @brief Update the state of a key from SDL event
   * @param key The SDL keycode
   * @param pressed True if key is pressed, false if released
   */
  void updateKey(SDL_Keycode key, bool pressed);

  /**
   * @brief Query if a key is currently pressed
   * @param key The SDL keycode to query
   * @return True if key is currently pressed
   */
  [[nodiscard]] bool isKeyPressed(SDL_Keycode key) const;

  /**
   * @brief Query if a key was just pressed this frame
   * @param key The SDL keycode to query
   * @return True if key was pressed this frame (before update() clears it)
   */
  [[nodiscard]] bool isKeyJustPressed(SDL_Keycode key) const;

  /**
   * @brief Query if a key is held down
   * @param key The SDL keycode to query
   * @return True if key is currently pressed
   * @note This is currently equivalent to isKeyPressed, provided for semantic clarity
   */
  [[nodiscard]] bool isKeyHeld(SDL_Keycode key) const;

  /**
   * @brief Get the duration a key has been held
   * @param key The SDL keycode to query
   * @return Duration in milliseconds (0 if key not pressed)
   */
  [[nodiscard]] std::chrono::milliseconds getKeyHoldDuration(SDL_Keycode key) const;

  /**
   * @brief Update frame timing and clear frame-specific flags
   * @param deltaTime Time elapsed since last update
   *
   * This advances the internal time and clears justPressed flags for all keys.
   * Call this once per frame after processing all SDL events.
   */
  void update(std::chrono::milliseconds deltaTime);

  /**
   * @brief Reset all key states
   *
   * Clears all tracked key states and resets internal time to zero.
   */
  void reset();

private:
  std::unordered_map<SDL_Keycode, KeyState> keyStates_;
  std::chrono::milliseconds currentTime_{0};
};

}  // namespace msd_gui

#endif  // INPUT_STATE_HPP
