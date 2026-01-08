# Design: Move Camera Controller to MSD-Sim

## Summary
This design moves camera motion control from msd-gui into msd-sim by creating a new `MotionController` component that encapsulates the movement and rotation logic currently in `CameraController`. The camera will reference the `ReferenceFrame` of a Platform's visual Object instead of owning its own frame. This enables future computer-controlled agents to control camera-equipped objects within the simulation environment.

## Architecture Changes

### PlantUML Diagram
See: `./0005_camera_controller_sim.puml`

### New Components

#### MotionController
- **Purpose**: Encapsulates 3D motion control logic (translation and rotation) for simulation objects based on input commands. Replaces msd-gui::CameraController with simulation-appropriate implementation.
- **Header location**: `msd/msd-sim/src/Environment/MotionController.hpp`
- **Source location**: `msd/msd-sim/src/Environment/MotionController.cpp`
- **Key interfaces**:
  ```cpp
  class MotionController {
  public:
      explicit MotionController(msd_sim::Angle rotSpeed = msd_sim::Angle::fromRadians(0.05),
                                float moveSpeed = 0.1f);
      ~MotionController() = default;

      MotionController(const MotionController&) = default;
      MotionController& operator=(const MotionController&) = default;
      MotionController(MotionController&&) noexcept = default;
      MotionController& operator=(MotionController&&) noexcept = default;

      // Update a ReferenceFrame based on input commands
      void updateTransform(ReferenceFrame& frame,
                           const InputCommands& commands,
                           std::chrono::milliseconds deltaTime);

      // Configuration
      void setMoveSpeed(float speed);
      void setRotationSpeed(msd_sim::Angle speed);
      void setSensitivity(float sensitivity);

      float getMoveSpeed() const;
      msd_sim::Angle getRotationSpeed() const;
      float getSensitivity() const;

  private:
      msd_sim::Angle rotSpeed_;
      float moveSpeed_;
      float sensitivity_;
  };
  ```
- **Dependencies**:
  - `ReferenceFrame` — Transform to update (non-owning reference parameter)
  - `InputCommands` — Input data structure
  - `Coordinate` — 3D vector math
  - `Angle` — Type-safe angle representation
- **Thread safety**: Not thread-safe (modifies ReferenceFrame in place)
- **Error handling**: No exceptions; caller responsible for valid parameters

### Modified Components

#### Platform
- **Current location**: `msd/msd-sim/src/Environment/Platform.hpp`
- **Changes required**:
  1. Add private member `MotionController motionController_`
  2. Add public getter method `MotionController& getMotionController()`
  3. Update constructor to initialize MotionController with default parameters
- **Rationale**: Platform is the entity type that owns agents and state in msd-sim. Adding MotionController as a private member with public getter provides encapsulation while allowing camera-equipped platforms to control their visual object's transform based on input commands.
- **Backward compatibility**: Existing Platform functionality unchanged; purely additive change

#### Camera3D
- **Current location**: `msd/msd-gui/src/Camera3D.hpp`
- **Changes required**:
  1. Change `frame_` member from owned `ReferenceFrame` to non-owning `std::reference_wrapper<ReferenceFrame>`
  2. Update constructor to accept `ReferenceFrame& referenceFrame` instead of constructing its own
  3. Update `getReferenceFrame()` to return reference from wrapper
- **Rationale**: Camera should observe the transform of its attached object rather than own its own transform. This allows simulation logic to control camera position.
- **Backward compatibility**: Breaking change to Camera3D construction. Callers must now provide a ReferenceFrame reference.

#### SDLApplication
- **Current location**: `msd/msd-gui/src/SDLApp.hpp`
- **Changes required**:
  1. Remove `cameraController_` member (unique_ptr<CameraController>)
  2. Update `setupInputBindings()` to bind movement keys to InputCommands population
  3. Add method `updatePlayerInput()` to propagate InputCommands to Engine via `Engine::setPlayerInputCommands()`
  4. Create a player Platform in Engine during initialization with designated player ID
  5. Link Camera3D to player Platform's visual Object's ReferenceFrame
- **Rationale**: SDLApplication coordinates GUI input with simulation. It now translates input to InputCommands and passes them to the simulation layer via Engine's clean API.
- **Backward compatibility**: Internal change; no external API impact

#### Engine
- **Current location**: `msd/msd-sim/src/Engine/Engine.hpp`
- **Changes required**:
  1. Add player Platform ID tracking member `std::optional<int> playerPlatformId_`
  2. Add method `void setPlayerPlatformId(int id)` to designate player Platform
  3. Add method `void setPlayerInputCommands(const InputCommands& commands)` to forward input to player Platform
  4. Implement input forwarding: lookup player Platform by ID and call `platform.getMotionController().updateTransform(...)`
- **Rationale**: Engine encapsulates simulation state and provides clean API for GUI layer to control player Platform without exposing internal structure.
- **Backward compatibility**: Purely additive change; no existing functionality affected

### Integration Points

| New Component | Existing Component | Integration Type | Notes |
|---------------|-------------------|------------------|-------|
| MotionController | Platform | Composition | Platform owns MotionController by value (private member) |
| MotionController | ReferenceFrame | Parameter dependency | MotionController updates ReferenceFrame via non-owning reference |
| MotionController | InputCommands | Data dependency | Reads boolean flags to determine motion |
| Camera3D | ReferenceFrame | Non-owning reference | Camera references Platform's Object's ReferenceFrame |
| SDLApplication | Engine | Method call | Calls Engine::setPlayerInputCommands() to pass input |
| Engine | Platform | ID-based lookup | Engine tracks player Platform ID and forwards input |

## Implementation Details

### MotionController Logic Transfer

The `updateTransform()` method will implement the same logic currently in `CameraController::updateFromInput()`:

**Linear Movement** (from local to global frame):
- `moveForward` → Translate -Z in local frame (forward)
- `moveBackward` → Translate +Z in local frame (backward)
- `moveLeft` → Translate -X in local frame (left)
- `moveRight` → Translate +X in local frame (right)
- `moveUp` → Translate +Y in local frame (up)
- `moveDown` → Translate -Y in local frame (down)

**Rotation** (update EulerAngles):
- `pitchUp` → Increase pitch angle
- `pitchDown` → Decrease pitch angle
- `yawLeft` → Increase yaw angle
- `yawRight` → Decrease yaw angle

All movements scaled by `moveSpeed_ * sensitivity_` and `deltaTime` for frame-rate independence.

### Camera3D Reference Frame Binding

**Current flow**:
```
Camera3D owns ReferenceFrame
CameraController modifies Camera3D's ReferenceFrame
```

**New flow**:
```
Platform owns MotionController
Platform has optional link to Object (visual representation)
Object owns ReferenceFrame (transform)
MotionController updates Object's ReferenceFrame via Platform
Camera3D references Object's ReferenceFrame (non-owning)
```

**Implementation**:
1. SDLApplication creates player Platform in Engine during init
2. SDLApplication designates player Platform via `Engine::setPlayerPlatformId(id)`
3. Player Platform spawns visual Object in WorldModel
4. Platform links to visual Object via `setVisualObject()`
5. Camera3D constructed with reference to Platform's visual Object's ReferenceFrame
6. Input system populates InputCommands from keyboard state
7. SDLApplication calls `Engine::setPlayerInputCommands(commands)`
8. Engine looks up player Platform by ID and calls `platform.getMotionController().updateTransform(visualObject_.getTransform(), commands, deltaTime)`
9. Camera3D reads updated transform for view matrix computation

### Input Command Flow

```
SDL Keyboard Events
    ↓
InputHandler::handleSDLEvent()
    ↓
InputState (key press tracking)
    ↓
SDLApplication::updatePlayerInput()
    ↓
InputCommands population (bool flags)
    ↓
Engine::setPlayerInputCommands(InputCommands)
    ↓
Engine looks up player Platform by ID
    ↓
Player Platform::getMotionController()
    ↓
MotionController::updateTransform(Object::ReferenceFrame, commands, deltaTime)
    ↓
Object::ReferenceFrame updated
    ↓
Camera3D::getViewMatrix() (reads updated transform)
```

## Test Impact

### Existing Tests Affected

| Test File | Test Case | Impact | Action Required |
|-----------|-----------|--------|------------------|
| `msd-gui/test/unit/camera_controller_test.cpp` | All tests | Removed | Delete test file (functionality moved to msd-sim) |
| `msd-gui/test/integration/sdl_app_test.cpp` | Camera movement tests | Modified | Update to use new Platform-based camera control |

### New Tests Required

#### Unit Tests

| Component | Test Case | What It Validates |
|-----------|-----------|-------------------|
| MotionController | `updateTransform_moveForward_translatesInLocalZ` | Forward movement applies -Z translation in local frame |
| MotionController | `updateTransform_pitchUp_increasesEulerPitch` | Pitch rotation increases pitch angle |
| MotionController | `updateTransform_scaledByDeltaTime_frameRateIndependent` | Movement scales properly with delta time |
| MotionController | `setMoveSpeed_updatesSpeed_affectsTranslation` | Speed configuration affects movement distance |
| MotionController | `updateTransform_withSensitivity_scalesMovement` | Sensitivity multiplier affects movement |
| Platform | `getMotionController_returnsReference` | Platform provides access to MotionController |

#### Integration Tests

| Test Case | Components Involved | What It Validates |
|-----------|---------------------|-------------------|
| `Platform_MotionController_Object_integration` | Platform, MotionController, Object, ReferenceFrame | MotionController correctly updates Platform's visual Object transform |
| `Camera_Platform_sync_integration` | Camera3D, Platform, Object | Camera view matrix updates when Platform moves |
| `SDLApp_input_to_motion_integration` | SDLApplication, InputHandler, Platform, MotionController | Input events propagate through to Platform motion |

## Design Decisions (Resolved)

All open questions have been resolved based on human feedback:

1. **MotionController ownership in Platform** → **DECISION**: Private member with public getter
   - Platform will have `private: MotionController motionController_`
   - Access via `public: MotionController& getMotionController()`
   - Rationale: Better encapsulation while maintaining clean access

2. **Engine method for player input** → **DECISION**: Add `Engine::setPlayerInputCommands(InputCommands)`
   - Engine will track player Platform ID
   - Engine provides clean API: `setPlayerPlatformId(int)` and `setPlayerInputCommands(const InputCommands&)`
   - Rationale: Encapsulates player Platform access, prevents GUI from accessing internal Engine structure

3. **Camera3D construction change impact** → **DECISION**: Breaking change only (no backward compatibility)
   - Camera3D constructor will require `ReferenceFrame&` parameter
   - No deprecated constructor provided
   - Rationale: Project is in early development; clean break is simpler than maintaining legacy paths

4. **AC4 Interpretation** → **CONFIRMED**: Camera requires a platform/reference frame for construction
   - Camera3D must receive a ReferenceFrame reference at construction
   - Typically this will be Platform's visual Object's ReferenceFrame
   - Could be any ReferenceFrame for flexibility

5. **Movement commands mapping** → **CONFIRMED**: Assumption correct
   - MotionController consumes InputCommands (boolean flags)
   - SDLApp maps keyboard keys to InputCommands
   - Clear separation: GUI handles input mapping, sim handles motion logic

6. **Platform-Camera relationship** → **CONFIRMED**: Designated player Platform using explicit ID
   - Engine will track player Platform ID explicitly
   - GUI designates player Platform via `Engine::setPlayerPlatformId(int)`
   - Engine forwards input commands to player Platform by ID lookup

### Prototype Required
None — This is a refactoring of existing functionality with clear requirements and resolved design decisions.
