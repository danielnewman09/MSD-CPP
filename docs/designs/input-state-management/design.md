# Design: Input State Tracking and Management System

## Summary

This design introduces a comprehensive input management system that separates input state tracking from input handling logic, enabling flexible control of both the camera and simulation objects via keyboard input. The system supports three input modes (Continuous, TriggerOnce, Interval) and provides a clean integration between `msd-gui` and `msd-sim` through an `InputControlAgent` that implements the existing `BaseAgent` interface. This architecture addresses the current limitation where `SDLApp` naively processes keypresses directly in the event loop, which does not scale for complex behaviors or support different input semantics (e.g., "hold to move" vs "press once to jump").

## Architecture Changes

### PlantUML Diagram
See: `./input-state-management.puml`

### New Components

#### InputState (msd-gui)
- **Purpose**: Tracks the current state of all keyboard inputs with timestamp information for duration-based queries
- **Header location**: `msd/msd-gui/src/InputState.hpp`
- **Source location**: `msd/msd-gui/src/InputState.cpp`
- **Key interfaces**:
  ```cpp
  struct KeyState {
      bool pressed{false};             // Currently pressed
      bool justPressed{false};         // Pressed this frame (cleared each update)
      std::chrono::milliseconds pressTime{0};        // When key was first pressed
      std::chrono::milliseconds lastTriggerTime{0};  // Last time action triggered (for interval mode)
  };

  class InputState {
  public:
      InputState() = default;
      ~InputState() = default;

      InputState(const InputState&) = default;
      InputState& operator=(const InputState&) = default;
      InputState(InputState&&) noexcept = default;
      InputState& operator=(InputState&&) noexcept = default;

      // Update from SDL events
      void updateKey(SDL_Keycode key, bool pressed);

      // Query methods
      bool isKeyPressed(SDL_Keycode key) const;
      bool isKeyJustPressed(SDL_Keycode key) const;
      bool isKeyHeld(SDL_Keycode key) const;
      std::chrono::milliseconds getKeyHoldDuration(SDL_Keycode key) const;

      // Frame management
      void update(std::chrono::milliseconds deltaTime);
      void reset();

  private:
      std::unordered_map<SDL_Keycode, KeyState> keyStates_;
      std::chrono::milliseconds currentTime_{0};
  };
  ```
- **Dependencies**: SDL3 (SDL_Keycode), chrono
- **Thread safety**: Not thread-safe (single-threaded GUI operation assumed)
- **Error handling**: Returns default values for unknown keys (no exceptions)

**Design rationale**:
- Separates input state from input handling logic
- Supports multiple query modes (pressed, just pressed, held, duration)
- `justPressed` flag cleared each frame via `update()` enables single-trigger semantics
- Timestamp tracking enables duration-based and interval-based behaviors
- Unordered map provides O(1) lookups for key state queries

#### InputMode (msd-gui)
- **Purpose**: Enum defining how input bindings should trigger
- **Header location**: `msd/msd-gui/src/InputHandler.hpp`
- **Key interfaces**:
  ```cpp
  enum class InputMode : uint8_t {
      Continuous,   // Trigger every frame while key is held
      TriggerOnce,  // Trigger only on initial press (ignore hold)
      Interval,     // Trigger at fixed intervals while key is held
      PressAndHold  // Trigger on release, recording the hold time as a percentage
                    // of some maximum hold time (e.g. if held for 1000ms with a 
                    // 2000ms maximum hold time, release with a 50% modifier)
  };
  ```
- **Dependencies**: None
- **Thread safety**: N/A (pure enum)
- **Error handling**: N/A

**Design rationale**:
- Provides semantic clarity for different input behaviors
- `Continuous`: Movement controls (WASD) - execute while held
- `TriggerOnce`: Jump, spawn object - execute once per press
- `Interval`: Auto-fire weapons, repeated actions - execute periodically
- `PressAndHold`: Throw an object a distance based on strength

#### InputBinding (msd-gui)
- **Purpose**: Binds a key to an action with specified input mode
- **Header location**: `msd/msd-gui/src/InputHandler.hpp`
- **Key interfaces**:
  ```cpp
  class InputBinding {
  public:
      InputBinding(SDL_Keycode key,
                   InputMode mode,
                   std::function<void()> action,
                   std::chrono::milliseconds interval = std::chrono::milliseconds{0});

      InputBinding(const InputBinding&) = default;
      InputBinding& operator=(const InputBinding&) = default;
      InputBinding(InputBinding&&) noexcept = default;
      InputBinding& operator=(InputBinding&&) noexcept = default;
      ~InputBinding() = default;

      SDL_Keycode getKey() const { return key_; }
      InputMode getMode() const { return mode_; }

      // Check if binding should trigger given current input state
      bool shouldTrigger(const InputState& state);

      // Execute the bound action
      void execute();

  private:
      SDL_Keycode key_;
      InputMode mode_;
      std::chrono::milliseconds intervalMs_{0};
      std::function<void()> action_;
      std::chrono::milliseconds lastExecuteTime_{0};
  };
  ```
- **Dependencies**: InputState, InputMode, SDL3, chrono, functional
- **Thread safety**: Not thread-safe (assumes single-threaded execution)
- **Error handling**: No exceptions; action execution failure depends on bound function

**Design rationale**:
- Encapsulates the binding logic separate from state tracking
- `shouldTrigger()` implements mode-specific logic (continuous, once, interval)
- Stores last execution time for interval mode
- Uses `std::function` for flexibility (lambdas, free functions, member functions via bind)
- Interval mode requires explicit interval duration in constructor

#### InputHandler (msd-gui)
- **Purpose**: Manages collection of input bindings and processes them based on InputState
- **Header location**: `msd/msd-gui/src/InputHandler.hpp`
- **Source location**: `msd/msd-gui/src/InputHandler.cpp`
- **Key interfaces**:
  ```cpp
  class InputHandler {
  public:
      InputHandler();
      ~InputHandler() = default;

      // Delete copy (owns unique state)
      InputHandler(const InputHandler&) = delete;
      InputHandler& operator=(const InputHandler&) = delete;

      // Allow move
      InputHandler(InputHandler&&) noexcept = default;
      InputHandler& operator=(InputHandler&&) noexcept = default;

      // Binding management
      void addBinding(InputBinding binding);
      void removeBinding(SDL_Keycode key);
      void clearBindings();

      // Event handling
      void handleSDLEvent(const SDL_Event& event);

      // Frame update
      void update(std::chrono::milliseconds deltaTime);

      // Process all bindings (call after update)
      void processInput();

      // State access
      const InputState& getInputState() const { return inputState_; }

  private:
      InputState inputState_;
      std::vector<InputBinding> bindings_;
  };
  ```
- **Dependencies**: InputState, InputBinding, SDL3
- **Thread safety**: Not thread-safe
- **Error handling**: No exceptions from public interface

**Design rationale**:
- Central coordination point for input processing
- Owns InputState (single source of truth)
- `handleSDLEvent()` updates state from SDL events
- `update()` advances time and clears frame-specific flags
- `processInput()` evaluates all bindings and executes triggered actions
- Separation of `update()` and `processInput()` allows state to be queried independently

**Typical usage flow**:
```cpp
// In event loop
while (SDL_PollEvent(&event)) {
    inputHandler.handleSDLEvent(event);
}

// After event processing
inputHandler.update(deltaTime);
inputHandler.processInput();
```

#### CameraController (msd-gui)
- **Purpose**: Encapsulates camera movement logic based on InputState
- **Header location**: `msd/msd-gui/src/CameraController.hpp`
- **Source location**: `msd/msd-gui/src/CameraController.cpp`
- **Key interfaces**:
  ```cpp
  class CameraController {
  public:
      explicit CameraController(Camera3D& camera,
                                float moveSpeed = 0.1f,
                                Angle rotSpeed = Angle::fromRadians(0.05));

      CameraController(const CameraController&) = delete;
      CameraController& operator=(const CameraController&) = delete;
      CameraController(CameraController&&) noexcept = default;
      CameraController& operator=(CameraController&&) noexcept = default;
      ~CameraController() = default;

      // Update camera from input state
      void updateFromInput(const InputState& inputState,
                           std::chrono::milliseconds deltaTime);

      // Configuration
      void setMoveSpeed(float speed) { moveSpeed_ = speed; }
      void setRotationSpeed(Angle speed) { rotSpeed_ = speed; }
      void setSensitivity(float sensitivity) { sensitivity_ = sensitivity; }

      float getMoveSpeed() const { return moveSpeed_; }
      Angle getRotationSpeed() const { return rotSpeed_; }

  private:
      Camera3D& camera_;               // Non-owning reference
      float moveSpeed_{0.1f};          // Units per frame
      msd_sim::Angle rotSpeed_{0.05};  // Radians per frame
      float sensitivity_{1.0f};        // Multiplier for input
  };
  ```
- **Dependencies**: Camera3D, InputState, msd_sim::Angle
- **Thread safety**: Not thread-safe
- **Error handling**: No exceptions

**Design rationale**:
- Separates camera control logic from SDLApp (single responsibility)
- Non-owning reference to Camera3D (camera owned by GPUManager)
- `updateFromInput()` reads state but doesn't modify it (InputHandler owns state)
- Delta-time scaling can be added to `updateFromInput()` for frame-rate independence
- Configuration methods allow runtime adjustment of movement speed/sensitivity

**Camera movement mapping**:
- W/S: Move forward/backward in camera's local Z direction
- A/D: Move left/right in camera's local X direction
- Q/E: Move up/down in camera's local Y direction
- Arrow Up/Down: Pitch camera
- Arrow Left/Right: Yaw camera

#### InputCommands (msd-sim)
- **Purpose**: Plain data structure representing current input command state for agent control
- **Header location**: `msd/msd-sim/src/Agent/InputCommands.hpp`
- **Key interfaces**:
  ```cpp
  struct InputCommands {
      // Linear movement
      bool moveForward{false};
      bool moveBackward{false};
      bool moveLeft{false};
      bool moveRight{false};
      bool moveUp{false};
      bool moveDown{false};

      // Rotation
      bool pitchUp{false};
      bool pitchDown{false};
      bool yawLeft{false};
      bool yawRight{false};
      bool rollLeft{false};
      bool rollRight{false};

      // Discrete actions
      bool jump{false};

      // Helper to reset all commands
      void reset() {
          moveForward = moveBackward = moveLeft = moveRight = false;
          moveUp = moveDown = false;
          pitchUp = pitchDown = yawLeft = yawRight = false;
          rollLeft = rollRight = false;
          jump = false;
      }
  };
  ```
- **Dependencies**: None
- **Thread safety**: Value type (safe to copy)
- **Error handling**: N/A (plain data)

**Design rationale**:
- Pure data structure (no logic)
- Created from InputState in msd-gui, consumed in msd-sim
- Decouples input representation from simulation logic
- Boolean flags for simplicity (velocity/force calculation in agent)
- Extensible for future commands (attack, interact, etc.)

#### InputControlAgent (msd-sim)
- **Purpose**: Agent implementation that translates InputCommands into InertialState updates
- **Header location**: `msd/msd-sim/src/Agent/InputControlAgent.hpp`
- **Source location**: `msd/msd-sim/src/Agent/InputControlAgent.cpp`
- **Key interfaces**:
  ```cpp
  class InputControlAgent : public BaseAgent {
  public:
      explicit InputControlAgent(double maxSpeed = 10.0,
                                 Angle maxAngularSpeed = Angle::fromRadians(1.0));

      ~InputControlAgent() override = default;

      InputControlAgent(const InputControlAgent&) = delete;
      InputControlAgent& operator=(const InputControlAgent&) = delete;
      InputControlAgent(InputControlAgent&&) noexcept = default;
      InputControlAgent& operator=(InputControlAgent&&) noexcept = default;

      // BaseAgent interface
      InertialState updateState(const InertialState& currentState) override;

      // Input interface
      void setInputCommands(const InputCommands& commands) { inputCommands_ = commands; }
      const InputCommands& getInputCommands() const { return inputCommands_; }

      // Configuration
      void setMaxSpeed(double speed) { maxSpeed_ = speed; }
      void setMaxAngularSpeed(Angle speed) { maxAngularSpeed_ = speed; }

  private:
      InputCommands inputCommands_;
      double maxSpeed_{10.0};                              // m/s
      msd_sim::Angle maxAngularSpeed_{Angle::fromRadians(1.0)};  // rad/s
  };
  ```
- **Dependencies**: BaseAgent, InputCommands, InertialState, Coordinate, Angle
- **Thread safety**: Not thread-safe
- **Error handling**: No exceptions from updateState()

**Design rationale**:
- Implements existing BaseAgent interface (fits cleanly into simulation architecture)
- Separates input commands from state update logic
- `setInputCommands()` called from Engine when GUI updates input
- `updateState()` translates boolean commands into velocity/acceleration updates
- Max speed limits prevent unrealistic motion
- Commands are directional (agent converts to velocity in object's local frame)

**Update logic**:
```cpp
InertialState InputControlAgent::updateState(const InertialState& currentState) {
    InertialState newState = currentState;

    // Calculate velocity based on input commands
    Coordinate velocity{0, 0, 0};
    if (inputCommands_.moveForward) velocity.x() += maxSpeed_;
    if (inputCommands_.moveBackward) velocity.x() -= maxSpeed_;
    if (inputCommands_.moveRight) velocity.y() += maxSpeed_;
    if (inputCommands_.moveLeft) velocity.y() -= maxSpeed_;
    if (inputCommands_.moveUp) velocity.z() += maxSpeed_;
    if (inputCommands_.moveDown) velocity.z() -= maxSpeed_;

    newState.velocity = velocity;
    // ... similar for angular velocity ...

    return newState;
}
```

### Modified Components

#### SDLApplication (msd-gui)
- **Current location**: `msd/msd-gui/src/SDLApp.hpp`, `msd/msd-gui/src/SDLApp.cpp`
- **Changes required**:
  1. Add member variables:
     ```cpp
     std::unique_ptr<InputHandler> inputHandler_;
     std::unique_ptr<CameraController> cameraController_;
     ```
  2. Replace direct event handling in `handleEvents()`:
     ```cpp
     void handleEvents() {
         SDL_Event event;
         while (SDL_PollEvent(&event)) {
             if (event.type == SDL_EVENT_QUIT) {
                 status_ = Status::Exiting;
             }
             inputHandler_->handleSDLEvent(event);
         }

         inputHandler_->update(frameDeltaTime_);
         inputHandler_->processInput();

         // Update camera via controller
         cameraController_->updateFromInput(inputHandler_->getInputState(), frameDeltaTime_);

         // Update player object agent (if present)
         updatePlayerInput();
     }
     ```
  3. Add `setupInputBindings()` method to configure key bindings:
     ```cpp
     void setupInputBindings() {
         // Example: Z key spawns pyramid (TriggerOnce mode)
         inputHandler_->addBinding(InputBinding{
             SDLK_Z,
             InputMode::TriggerOnce,
             [this]() { spawnRandomObject("pyramid"); }
         });

         // Example: X key removes object (TriggerOnce mode)
         inputHandler_->addBinding(InputBinding{
             SDLK_X,
             InputMode::TriggerOnce,
             [this]() { removeLastObject(); }
         });
     }
     ```
  4. Add `updatePlayerInput()` method to propagate input to simulation:
     ```cpp
     void updatePlayerInput() {
         const auto& inputState = inputHandler_->getInputState();

         // Convert InputState to InputCommands
         InputCommands commands;
         commands.moveForward = inputState.isKeyPressed(SDLK_W);
         commands.moveBackward = inputState.isKeyPressed(SDLK_S);
         commands.moveLeft = inputState.isKeyPressed(SDLK_A);
         commands.moveRight = inputState.isKeyPressed(SDLK_D);
         commands.moveUp = inputState.isKeyPressed(SDLK_Q);
         commands.moveDown = inputState.isKeyPressed(SDLK_E);
         // ... etc ...

         // Send to engine
         engine_.setPlayerInputCommands(commands);
     }
     ```
  5. Remove hardcoded movement constants (moveSpeed_, unitX_, etc.) - now in CameraController
- **Backward compatibility**: Existing keyboard controls will continue to work via new input system

#### Platform (msd-sim)
- **Current location**: `msd/msd-sim/src/Environment/Platform.hpp`, `msd/msd-sim/src/Environment/Platform.cpp`
- **Existing structure**: Platform already owns `std::unique_ptr<BaseAgent> agent_` and `InertialState state_`
- **Changes required**:
  1. Add optional Object reference for visual representation:
     ```cpp
     std::optional<std::reference_wrapper<Object>> visualObject_;
     ```
  2. Add methods to link Platform to an Object in WorldModel:
     ```cpp
     void setVisualObject(Object& object) { visualObject_ = object; }
     bool hasVisualObject() const { return visualObject_.has_value(); }
     Object& getVisualObject() { return visualObject_->get(); }
     const Object& getVisualObject() const { return visualObject_->get(); }
     ```
  3. Add agent accessor methods:
     ```cpp
     void setAgent(std::unique_ptr<BaseAgent> agent) { agent_ = std::move(agent); }
     BaseAgent* getAgent() { return agent_.get(); }
     const BaseAgent* getAgent() const { return agent_.get(); }
     bool hasAgent() const { return agent_ != nullptr; }
     ```
  4. Add state accessor:
     ```cpp
     const InertialState& getState() const { return state_; }
     InertialState& getState() { return state_; }
     ```
  5. Update `update()` to synchronize visual object position with Platform state:
     ```cpp
     void update(const std::chrono::milliseconds& currTime) {
         // Calculate delta time
         auto deltaTime = currTime - lastUpdateTime_;

         // Update state via agent if present
         if (agent_) {
             state_ = agent_->updateState(state_);
         }

         // Sync visual object position if linked
         if (visualObject_.has_value()) {
             visualObject_->get().setPosition(state_.position);
             visualObject_->get().getTransform().setRotation(state_.angularPosition);
         }

         lastUpdateTime_ = currTime;
     }
     ```
- **Design rationale**:
  - Platform already owns agent - InputControlAgent fits naturally with existing architecture
  - Optional Object reference allows visual representation without tight coupling
  - Platform owns the simulation state (InertialState); Object is purely for rendering
  - Multiple Platforms can exist, each with its own agent and InputCommands (satisfies AC3: non-unique state)
  - Separation of concerns: Platform handles agent logic, Object handles rendering
- **Backward compatibility**: Existing Platform usage unchanged; new methods are additive

#### Engine (msd-sim)
- **Current location**: `msd/msd-sim/src/Engine.hpp`, `msd/msd-sim/src/Engine.cpp`
- **Changes required**:
  1. Add player Platform tracking:
     ```cpp
     std::optional<uint32_t> playerPlatformId_;
     ```
  2. Add method to set player input commands:
     ```cpp
     void setPlayerInputCommands(const InputCommands& commands) {
         if (!playerPlatformId_.has_value()) return;

         // Find platform by ID
         for (auto& platform : worldModel_.getPlatforms()) {
             if (platform.getId() == *playerPlatformId_) {
                 if (auto* agent = platform.getAgent()) {
                     if (auto* inputAgent = dynamic_cast<InputControlAgent*>(agent)) {
                         inputAgent->setInputCommands(commands);
                     }
                 }
                 break;
             }
         }
     }
     ```
  3. Add method to spawn player-controlled Platform with visual Object:
     ```cpp
     uint32_t spawnPlayerPlatform(const std::string& assetName,
                                  const Coordinate& position,
                                  const EulerAngles& orientation) {
         // Create visual object
         auto& asset = getAssetRegistry().getAsset(assetName)->get();
         auto object = Object::createGraphical(asset, ReferenceFrame{position, orientation});
         size_t objIndex = worldModel_.spawnObject(std::move(object));

         // Create Platform with InputControlAgent
         uint32_t platformId = worldModel_.getNextPlatformId();
         Platform platform{platformId};
         platform.setAgent(std::make_unique<InputControlAgent>());
         platform.getState().position = position;
         platform.getState().angularPosition = orientation;
         platform.setVisualObject(worldModel_.getObject(objIndex));

         worldModel_.addPlatform(std::move(platform));
         playerPlatformId_ = platformId;
         return platformId;
     }
     ```
- **Backward compatibility**: New methods additive; existing `spawnInertialObject()` unchanged

#### WorldModel (msd-sim)
- **Current location**: `msd/msd-sim/src/Environment/WorldModel.hpp`, `msd/msd-sim/src/Environment/WorldModel.cpp`
- **Changes required**:
  1. Add method to get next platform ID:
     ```cpp
     uint32_t getNextPlatformId() const { return nextPlatformId_++; }
     ```
  2. Add non-const accessor for platforms:
     ```cpp
     std::vector<Platform>& getPlatforms() { return platforms_; }
     ```
  3. Update `update()` to invoke Platform updates:
     ```cpp
     void update(std::chrono::milliseconds deltaTime) {
         // Update all platforms (agent logic + visual sync)
         for (auto& platform : platforms_) {
             platform.update(time_ + deltaTime);
         }

         // ... existing physics integration for Objects ...

         time_ += deltaTime;
     }
     ```
- **Backward compatibility**: Platform updates are additive; existing Object physics unchanged

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| InputHandler | SDLApplication | Composition (unique_ptr) | SDLApp owns InputHandler, calls handleSDLEvent/update/processInput |
| CameraController | Camera3D | Non-owning reference | CameraController references Camera3D owned by GPUManager |
| InputControlAgent | BaseAgent | Inheritance | Fits into existing agent architecture |
| InputCommands | Engine | Parameter passing | Created in SDLApp, passed to Engine, forwarded to agent |
| Platform.visualObject_ | Object | Non-owning reference | Platform references Object in WorldModel for visual sync |
| Platform.agent_ | BaseAgent | Composition (unique_ptr) | Platform owns agent (existing architecture) |

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| N/A | N/A | No existing tests for SDLApp | Create new tests |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| InputState | `updateKey_SetsKeyPressed` | Updating key sets pressed flag |
| InputState | `updateKey_Release_ClearsPressedFlag` | Releasing key clears pressed flag |
| InputState | `isKeyJustPressed_ClearedAfterUpdate` | justPressed flag cleared on update() |
| InputState | `getKeyHoldDuration_ReturnsCorrectDuration` | Duration calculation correct |
| InputBinding | `shouldTrigger_Continuous_TriggersWhileHeld` | Continuous mode triggers every frame |
| InputBinding | `shouldTrigger_TriggerOnce_TriggersOnlyOnce` | TriggerOnce ignores hold |
| InputBinding | `shouldTrigger_Interval_TriggersAtInterval` | Interval mode respects timing |
| InputHandler | `addBinding_StoresBinding` | Binding added to collection |
| InputHandler | `processInput_ExecutesTriggeredBindings` | Bindings execute when conditions met |
| CameraController | `updateFromInput_W_MovesCameraForward` | W key moves camera in local Z |
| CameraController | `updateFromInput_ArrowUp_PitchesCamera` | Arrow up increases pitch |
| InputControlAgent | `updateState_MoveForward_SetsPositiveXVelocity` | Forward command sets +X velocity |
| InputControlAgent | `updateState_NoCommands_ZeroVelocity` | No commands result in zero velocity |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `InputHandler_to_CameraController` | InputHandler, CameraController, Camera3D | Camera responds to input state |
| `SDLApp_to_Engine_InputPropagation` | SDLApp, Engine, InputControlAgent | Input commands reach agent in simulation |
| `Object_with_Agent_Updates` | WorldModel, Object, InputControlAgent | Objects with agents update via agent logic |

## Open Questions

### Design Decisions (Human Input Needed)

1. **Frame delta time tracking**
   - Option A: Add frame delta time tracking to SDLApplication (track time between frames)
   - Option B: Use fixed time step (assume 60 FPS → 16ms per frame)
   - Option C: Let caller pass delta time to `update()` methods
   - Recommendation: **Option A** — Frame-rate independence important for smooth movement
   - **Human decision needed**: Confirm preference **Response** Option A

2. **Player object lifecycle**
   - Option A: Engine automatically creates player object on construction
   - Option B: SDLApp explicitly spawns player object via `spawnPlayerObject()`
   - Option C: Player object concept removed; camera is only player representation
   - Recommendation: **Option C** for initial implementation (defer player object to future ticket)
   - **Human decision needed**: Is player-controlled object needed for this ticket, or only camera control? **Response** Only worry about camera control for now.

3. **Input command propagation frequency**
   - Option A: Update agent input every frame (tight coupling)
   - Option B: Update agent input only when input changes (event-driven)
   - Option C: Update agent input at fixed simulation rate (decoupled)
   - Recommendation: **Option A** — Simplest, matches current architecture
   - **Human decision needed**: Confirm approach **Response** Use Option A for now, and note the tradeoffs

4. **Camera controller ownership**
   - Option A: SDLApp owns CameraController (as designed)
   - Option B: GPUManager owns CameraController (co-locate with Camera3D)
   - Recommendation: **Option A** — Input handling is application concern, not rendering concern
   - **Human decision needed**: Confirm ownership location **Response** Yes, Option A

### Prototype Required

None identified. The design leverages existing patterns (BaseAgent interface, value semantics for InputCommands) and follows established project conventions.

### Requirements Clarification

1. **Acceptance Criterion 3**: "The internal state shall be non-unique in the msd-simulation library (multiple agents/actors can each have their own internal state)"
   - **Interpretation**: Each Platform instance has its own `InputControlAgent` with its own `InputCommands` state
   - **Not** a singleton/shared state — multiple Platforms can coexist independently
   - **Status**: ✅ Addressed — Design supports multiple Platforms, each owning their own agent with independent InputCommands

2. **Acceptance Criterion 5**: "The input handler mechanism shall be customizable based on how the logic is invoked upon key press"
   - **Interpretation**: Mode specified when binding created (compile-time), with extensibility for new modes
   - Current design supports four modes: Continuous, TriggerOnce, Interval, PressAndHold
   - **Status**: ✅ Addressed — InputMode enum + InputBinding class support customizable behavior

3. **Acceptance Criterion 7**: "The agent update logic shall be placed in the `Platform` Object in `msd-sim` (not `Object`)"
   - **Interpretation**: Platform (not Object) owns the agent and handles state updates
   - **Status**: ✅ Addressed — Design modifies Platform, not Object

4. **Thread safety requirements** (Non-Functional Requirement)
   - Ticket states: "Thread safety is not a big concern here and will be addressed in a future ticket"
   - **Decision**: Single-threaded operation sufficient for this ticket; no thread-safety additions

## Implementation Notes

### Implementation Order

1. **Phase 1: Input State Foundation** (msd-gui)
   - Implement `InputState` with KeyState tracking
   - Implement `InputMode` enum
   - Unit tests for InputState

2. **Phase 2: Input Handling Layer** (msd-gui)
   - Implement `InputBinding`
   - Implement `InputHandler`
   - Unit tests for InputBinding and InputHandler

3. **Phase 3: Camera Control** (msd-gui)
   - Implement `CameraController`
   - Integrate with SDLApplication
   - Unit tests for CameraController
   - Integration test for camera movement

4. **Phase 4: Simulation Integration** (msd-sim)
   - Implement `InputCommands` struct
   - Implement `InputControlAgent`
   - Extend `Platform` with visual object reference and accessor methods
   - Unit tests for InputControlAgent
   - Unit tests for Platform extensions

5. **Phase 5: Engine Integration** (msd-sim + msd-gui)
   - Modify `Engine` for input command propagation to Platforms
   - Modify `WorldModel` to invoke Platform updates
   - Update `SDLApplication` to propagate input to engine
   - Integration tests for Platform-based input flow

6. **Phase 6: Migration and Cleanup**
   - Migrate existing key bindings to new system
   - Remove old input handling code from SDLApp
   - Update documentation

### File Creation Checklist

**New files**:
- `msd/msd-gui/src/InputState.hpp`
- `msd/msd-gui/src/InputState.cpp`
- `msd/msd-gui/src/InputHandler.hpp`
- `msd/msd-gui/src/InputHandler.cpp`
- `msd/msd-gui/src/CameraController.hpp`
- `msd/msd-gui/src/CameraController.cpp`
- `msd/msd-sim/src/Agent/InputCommands.hpp`
- `msd/msd-sim/src/Agent/InputControlAgent.hpp`
- `msd/msd-sim/src/Agent/InputControlAgent.cpp`

**Test files**:
- `msd/msd-gui/test/unit/InputStateTest.cpp`
- `msd/msd-gui/test/unit/InputHandlerTest.cpp`
- `msd/msd-gui/test/unit/CameraControllerTest.cpp`
- `msd/msd-sim/test/Agent/InputControlAgentTest.cpp`
- `msd/msd-sim/test/Environment/PlatformTest.cpp` (extend existing or create)
- `msd/msd-gui/test/integration/InputToCameraIntegrationTest.cpp`
- `msd/msd-sim/test/integration/PlatformAgentIntegrationTest.cpp`

**Modified files**:
- `msd/msd-gui/src/SDLApp.hpp`
- `msd/msd-gui/src/SDLApp.cpp`
- `msd/msd-sim/src/Environment/Platform.hpp`
- `msd/msd-sim/src/Environment/Platform.cpp`
- `msd/msd-sim/src/Environment/WorldModel.hpp`
- `msd/msd-sim/src/Environment/WorldModel.cpp`
- `msd/msd-sim/src/Engine.hpp`
- `msd/msd-sim/src/Engine.cpp`

### Build System Changes

**CMakeLists.txt updates**:
- Add new source files to `msd-gui` target
- Add new source files to `msd-sim` target
- Add test executables
- No new external dependencies required (uses existing SDL3, chrono, functional)

### Performance Considerations

1. **InputState lookup**: O(1) via unordered_map
2. **Binding evaluation**: O(n) where n = number of bindings (typically small, <20)
3. **Memory overhead**: Minimal (~100 bytes per key state, ~50 bytes per binding)
4. **Frame rate impact**: Negligible (<1% CPU for typical usage)

### Error Scenarios and Handling

| Scenario | Handling Strategy |
|----------|-------------------|
| Unknown key queried from InputState | Return default (false/zero) |
| Binding action throws exception | Propagate (let application handle) |
| Agent returns invalid InertialState | Validation in physics integration (future work) |
| Player Platform ID invalid | Check before access, log warning |
| Dynamic cast to InputControlAgent fails | Silently skip (Platform has different agent type) |
| Visual Object reference becomes invalid | Platform checks `hasVisualObject()` before sync |
