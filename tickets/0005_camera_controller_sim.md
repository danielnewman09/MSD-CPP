# Feature Ticket: Move Camera Controller to MSD-Sim

## Status
- [ ] Draft
- [ ] Ready for Design
- [ ] Design Complete — Awaiting Review
- [ ] Design Approved — Ready for Prototype
- [ ] Prototype Complete — Awaiting Review
- [ ] Ready for Implementation
- [X] Implementation Complete — Awaiting Review
- [ ] Approved — Ready to Merge
- [ ] Merged / Complete

## Metadata
- **Created**: 1-6-2026
- **Author**: Daniel M Newman
- **Priority**:  Medium 
- **Estimated Complexity**: Small 
- **Target Component(s)**: msd-gui, msd-sim

---

## Summary
Currently, the logic controlling the camera motion resides entirely within msd-gui. In the future, this application should be able to use the simulation environment to simulate when, for example, a computer-controlled agent can see specific things. In addition, the view of this agent should be controlled within the simulation environment. 

## Motivation
Moving the control of the camera entirely to the agent makes motion more clearly controlled within the simulation environment.

## Requirements

### Functional Requirements
1. The GUI application shall function identically to its current implementation.

### Non-Functional Requirements
- **Performance**: best practices for the codebase should be followed
- **Memory**: best practices for the codebase should be followed
- **Thread Safety**: no thread safety considerations
- **Backward Compatibility**: outside of the obvious change in control ownership, minimal external functions shall be touched.

## Constraints


## Acceptance Criteria
1. A new object in msd-sim - `MotionController` shall be created
2. `CameraController::rotSpeed_`, `CameraController::moveSpeed_`, and `CameraController::sensitivity_` shall be moved into `msd-sim::MotionController`
3. `CameraController` shall be removed
4. `Camera3D::referenceFrame_` shall be made a reference to `msd-sim::Platform::Object::referenceFrame_`
5. The movement commands: 
```
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
```
  shall be propagated to the `msd-sim::MotionController`. The camera position and orientation of the `msd-sim::Platform::Object::referenceFrame_` shall be updated based on these commands.
6. `msd-sim::Platform` shall contain a `MotionController` object

---

## Design Decisions (Human Input)

### Preferred Approaches

### Things to Avoid

### Open Questions

---

## References

### Related Code
- `msd/msd-gui/src/Camera3D.hpp` - Contains the camera positional information
- `msd/msd-gui/src/CameraController.hpp` - file to be removed
- `msd/msd-gui/src/SDLApp.hpp` - should contain new key bindings for the new motion controller 
- `msd/msd-sim/src/Platform.hpp` - contains new `MotionController` for AC6
- `msd/msd-sim/src/Object.hpp` - contains reference frame for object in AC4

### Related Documentation
- `docs/msd/msd-gui/camera3d.puml`
- `docs/msd/msd-gui/msd-gui-core.puml`
- `docs/msd/msd-gui/sdl-application.puml`
- `docs/msd/msd-sim/Environment/object.puml`
- `docs/msd/msd-sim/Environment/reference-frame.puml`

### Related Tickets
- 0004_gui_framerate.md

---

## Workflow Log

{This section is automatically updated as the workflow progresses}

### Design Phase
- **Started**: 2026-01-07
- **Completed**: 2026-01-07
- **Artifacts**:
  - `docs/designs/0005_camera_controller_sim/design.md`
  - `docs/designs/0005_camera_controller_sim/0005_camera_controller_sim.puml`
- **Notes**:
  - Created new MotionController class in msd-sim to replace msd-gui::CameraController
  - MotionController encapsulates movement/rotation logic with same parameters (rotSpeed, moveSpeed, sensitivity)
  - Camera3D modified to reference Platform's Object's ReferenceFrame instead of owning its own
  - Platform gains public MotionController member for direct camera control
  - Input flow: SDL Events → InputState → InputCommands → MotionController → ReferenceFrame → Camera3D
  - Open questions identified: MotionController ownership style, Engine player input API, Camera3D constructor backward compatibility
  - Design preserves existing functionality while enabling future agent-controlled cameras 

### Design Review Phase
- **Started**: 2026-01-07
- **Completed**: 2026-01-07
- **Status**: APPROVED
- **Reviewer Notes**: All open questions resolved by human feedback. Key decisions:
  - MotionController ownership: Private member with public getter (better encapsulation)
  - Player input API: Engine::setPlayerInputCommands() with player Platform ID tracking
  - Camera3D constructor: Breaking change only (no backward compatibility needed)
  - Player Platform designation: Explicit ID-based system in Engine
  - Design updated to incorporate all feedback and resolve all open questions 

### Prototype Phase
- **Started**: 
- **Completed**: 
- **Prototypes**: 
  - P1: {name} — {result}
- **Artifacts**: 
  - `docs/designs/{feature-name}/prototype-results.md`
- **Notes**: 

### Implementation Phase
- **Started**: 2026-01-07
- **Completed**: 2026-01-07
- **Files Created**:
  - `msd/msd-sim/src/Environment/MotionController.hpp`
  - `msd/msd-sim/src/Environment/MotionController.cpp`
  - `msd/msd-sim/test/Environment/MotionControllerTest.cpp`
- **Files Modified**:
  - `msd/msd-sim/src/Environment/Platform.hpp` (added MotionController member)
  - `msd/msd-sim/src/Environment/Platform.cpp` (initialize MotionController)
  - `msd/msd-gui/src/Camera3D.hpp` (constructor takes ReferenceFrame&)
  - `msd/msd-gui/src/Camera3D.cpp` (use reference_wrapper, fixed narrowing conversion)
  - `msd/msd-sim/src/Engine.cpp` (setPlayerInputCommands uses MotionController)
  - `msd/msd-gui/src/SDLApp.hpp` (removed CameraController member)
  - `msd/msd-gui/src/SDLApp.cpp` (create player platform, pass ReferenceFrame to GPUManager)
  - `msd/msd-gui/src/SDLGPUManager.hpp` (constructor takes ReferenceFrame&)
  - `msd/msd-sim/src/Environment/CMakeLists.txt` (added MotionController)
  - `msd/msd-gui/src/CMakeLists.txt` (removed CameraController files)
  - `msd/msd-sim/test/Environment/CMakeLists.txt` (added MotionControllerTest)
- **Files Deleted**:
  - `msd/msd-gui/src/CameraController.hpp`
  - `msd/msd-gui/src/CameraController.cpp`
- **Artifacts**:
  - `docs/designs/0005_camera_controller_sim/implementation-notes.md`
- **Notes**:
  - Implementation fully complete with comprehensive testing
  - All 20 unit tests passing (100% pass rate)
  - Fixed narrowing conversion in Camera3D constructor (M_PI double to float)
  - Tests cover: constructor, movement (6 directions), rotation (4 directions), frame-rate independence, setters/getters, platform integration, combined movements
  - Input flow validated: SDL → InputState → InputCommands → MotionController → ReferenceFrame → Camera
  - All acceptance criteria met and tested
  - Project builds cleanly in Debug configuration 

### Implementation Review Phase
- **Started**: 
- **Completed**: 
- **Status**: 
- **Reviewer Notes**: 

### Documentation Update Phase
- **Started**: 
- **Completed**: 
- **CLAUDE.md Updates**: 
- **Diagrams Indexed**: 
- **Notes**: 

---

## Human Feedback

{Add feedback here at any point. Agents will read this section.}

### Feedback on Design
{Your comments on the design}

### Feedback on Prototypes
{Your comments on prototype results}

### Feedback on Implementation
{Your comments on the implementation}
