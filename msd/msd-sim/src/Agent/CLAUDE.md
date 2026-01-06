# Agent Module Architecture Guide

> This document provides architectural context for AI assistants and developers.
> It references PlantUML diagrams in `docs/msd/msd-sim/` for detailed component relationships.

## Project Overview

**Agent** is a module within `msd-sim` that defines the interface for autonomous control logic. It provides an abstract base class that enables polymorphic agent behavior following the Strategy pattern.

## Architecture Overview

### High-Level Architecture

The Agent module provides a simple abstraction for control algorithms:

```
Platform
    └── BaseAgent (abstract)
            ├── ConcreteAgentA (user-defined)
            ├── ConcreteAgentB (user-defined)
            └── ... (extensible)
```

### Core Components

| Component | Location | Purpose | Diagram |
|-----------|----------|---------|---------|
| BaseAgent | `BaseAgent.hpp` | Abstract interface for control logic | [`msd-sim-core.puml`](../../../../../docs/msd/msd-sim/msd-sim-core.puml) |
| InputControlAgent | `InputControlAgent.hpp` | Human input control implementation | [`input-state-management.puml`](../../../../../docs/designs/input-state-management/input-state-management.puml) |
| InputCommands | `InputCommands.hpp` | Plain data struct bridging GUI and sim | [`input-state-management.puml`](../../../../../docs/designs/input-state-management/input-state-management.puml) |

---

## Component Details

### BaseAgent

**Location**: `BaseAgent.hpp`
**Type**: Header-only, abstract interface

#### Purpose
Pure virtual base class that defines the interface for autonomous control logic. Enables the Strategy pattern where different control algorithms can be swapped at runtime.

#### Key Interfaces
```cpp
class BaseAgent {
public:
  virtual ~BaseAgent() = default;

  /**
   * @brief Update the state based on current state.
   *
   * Implementations should compute and return the new desired state
   * based on the current kinematic state.
   *
   * @param currentState Current kinematic state of the platform
   * @return New desired kinematic state
   */
  virtual InertialState updateState(const InertialState& currentState) = 0;
};
```

#### Usage Example
```cpp
// Define a custom agent
class LinearMotionAgent : public BaseAgent {
public:
  InertialState updateState(const InertialState& currentState) override {
    InertialState newState = currentState;
    // Apply constant velocity in +X direction
    newState.velocity = Coordinate{1.0, 0.0, 0.0};
    return newState;
  }
};

// Assign to platform
Platform platform{0};
platform.setAgent(std::make_unique<LinearMotionAgent>());
```

#### Thread Safety
**Implementation-dependent** — The interface is pure virtual; thread safety depends on concrete implementations.

#### Memory Management
- Virtual destructor ensures proper cleanup of derived classes
- Owned by Platform via `std::unique_ptr<BaseAgent>`

---

### InputCommands

**Location**: `InputCommands.hpp`
**Type**: Header-only, plain data struct
**Diagram**: [`docs/designs/input-state-management/input-state-management.puml`](../../../../../docs/designs/input-state-management/input-state-management.puml)
**Introduced**: [Ticket: 0004_gui_framerate](../../../../../tickets/0004_gui_framerate.md)

#### Purpose
Plain data structure representing current input command state for agent control. Acts as the bridge between msd-gui (input source) and msd-sim (consumer), decoupling input representation from simulation logic.

#### Key Interfaces
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

  void reset();  // Reset all commands to false
};
```

#### Usage Example
```cpp
#include "msd-sim/src/Agent/InputCommands.hpp"

// In GUI layer
InputCommands commands;
commands.moveForward = inputState.isKeyPressed(SDLK_W);
commands.moveRight = inputState.isKeyPressed(SDLK_D);

// Pass to simulation
engine.setPlayerInputCommands(commands);

// In agent
void InputControlAgent::setInputCommands(const InputCommands& commands) {
  inputCommands_ = commands;
}
```

#### Thread Safety
**Value type** — Safe to copy across threads.

#### Memory Management
Value semantics with boolean fields only (no dynamic memory).

#### Dependencies
None (pure data structure).

---

### InputControlAgent

**Location**: `InputControlAgent.hpp`, `InputControlAgent.cpp`
**Type**: Concrete agent implementation
**Diagram**: [`docs/designs/input-state-management/input-state-management.puml`](../../../../../docs/designs/input-state-management/input-state-management.puml)
**Introduced**: [Ticket: 0004_gui_framerate](../../../../../tickets/0004_gui_framerate.md)

#### Purpose
Agent implementation that translates InputCommands into InertialState updates for human-controlled platforms. Implements the BaseAgent interface to fit cleanly into the simulation architecture.

#### Key Interfaces
```cpp
class InputControlAgent : public BaseAgent {
public:
  explicit InputControlAgent(double maxSpeed = 10.0,
                             Angle maxAngularSpeed = Angle::fromRadians(1.0));

  ~InputControlAgent() override = default;

  // BaseAgent interface
  InertialState updateState(const InertialState& currentState) override;

  // Input interface
  void setInputCommands(const InputCommands& commands);
  const InputCommands& getInputCommands() const;

  // Configuration
  void setMaxSpeed(double speed);
  void setMaxAngularSpeed(Angle speed);
  double getMaxSpeed() const;
  Angle getMaxAngularSpeed() const;

private:
  InputCommands inputCommands_;
  double maxSpeed_{10.0};                              // m/s
  msd_sim::Angle maxAngularSpeed_{Angle::fromRadians(1.0)};  // rad/s
};
```

#### Usage Example
```cpp
#include "msd-sim/src/Agent/InputControlAgent.hpp"

// Create agent with custom max speeds
auto agent = std::make_unique<InputControlAgent>(15.0, Angle::fromRadians(2.0));

// Assign to platform
Platform platform{0};
platform.setAgent(std::move(agent));

// Update input commands (called from GUI/Engine)
InputCommands commands;
commands.moveForward = true;
commands.yawLeft = true;

auto* agentPtr = platform.getAgent();
if (auto* inputAgent = dynamic_cast<InputControlAgent*>(agentPtr)) {
  inputAgent->setInputCommands(commands);
}

// Simulation update (called from Platform::update)
InertialState newState = inputAgent->updateState(currentState);
```

#### Usage Flow
1. GUI calls `setInputCommands()` with current input state
2. Engine calls `updateState()` during simulation update
3. Agent translates boolean commands into velocity/acceleration

#### Thread Safety
**Not thread-safe** — Assumes single-threaded simulation.

#### Error Handling
No exceptions from `updateState()`.

#### Memory Management
- **Ownership**: Owned by Platform via `std::unique_ptr<BaseAgent>`
- **Value semantics**: InputCommands copied by value
- **Delete copy**: Non-copyable (unique ownership semantics)
- **Allow move**: Movable for transfer of ownership

#### Dependencies
- `BaseAgent` — Abstract interface
- `InputCommands` — Input data structure
- `InertialState` — Kinematic state representation
- `Coordinate` — 3D vector for velocity
- `Angle` — Type-safe angle for angular speed

---

## Design Patterns in Use

### Strategy Pattern
**Used in**: BaseAgent and derived classes
**Purpose**: Encapsulates control algorithms as interchangeable objects. Platforms can have different control behaviors without modifying Platform code.

### RAII
**Used in**: Platform ownership of agents
**Purpose**: Platform owns agent via `std::unique_ptr`, ensuring automatic cleanup.

---

## Cross-Cutting Concerns

### Error Handling Strategy
- Pure virtual function — no default implementation
- Derived classes responsible for their own error handling

### Memory Management
- Abstract class with virtual destructor
- Designed for ownership via `std::unique_ptr`
- No raw pointer ownership

### Thread Safety Conventions
- Interface only — thread safety is implementation-specific
- Platform assumes single-threaded agent updates

---

## Implementing Custom Agents

### Basic Agent Template
```cpp
#include "msd-sim/src/Agent/BaseAgent.hpp"

class MyCustomAgent : public msd_sim::BaseAgent {
public:
  InertialState updateState(const InertialState& currentState) override {
    InertialState newState = currentState;

    // Implement your control logic here
    // Example: Simple velocity controller
    newState.velocity.x() = targetVelocity_;
    newState.acceleration.x() = (targetVelocity_ - currentState.velocity.x()) / dt_;

    return newState;
  }

private:
  double targetVelocity_{10.0};  // m/s
  double dt_{0.016};             // 60 Hz
};
```

### Agent with Configuration
```cpp
class ConfigurableAgent : public msd_sim::BaseAgent {
public:
  struct Config {
    double maxSpeed{100.0};
    double maxAcceleration{10.0};
    double turnRate{0.1};
  };

  explicit ConfigurableAgent(const Config& config) : config_{config} {}

  InertialState updateState(const InertialState& currentState) override {
    InertialState newState = currentState;
    // Use config_ parameters...
    return newState;
  }

private:
  Config config_;
};
```

### Agent with External Input
```cpp
class TeleopAgent : public msd_sim::BaseAgent {
public:
  void setCommand(const Coordinate& velocity) {
    commandVelocity_ = velocity;
  }

  InertialState updateState(const InertialState& currentState) override {
    InertialState newState = currentState;
    newState.velocity = commandVelocity_;
    return newState;
  }

private:
  Coordinate commandVelocity_;
};
```

---

## Build & Configuration

### Build Requirements
- **C++ Standard**: C++20
- **Dependencies**:
  - Environment module (InertialState)

### Building This Component

This module is built as part of `msd-sim`:

```bash
# From project root
cmake --preset conan-debug
cmake --build --preset debug-sim-only
```

---

## Future Development

### Planned Features
- **Sensor Integration**: Agents that respond to sensor data
- **Path Planning**: Integration with path planning algorithms
- **Behavior Trees**: Complex behavior composition
- **Multi-Agent Coordination**: Inter-agent communication

### Extension Points
- Derive from BaseAgent for custom control algorithms
- Add sensor dependencies via constructor injection
- Implement state machines for complex behaviors

---

## Coding Standards

This module follows the project-wide coding standards defined in the [root CLAUDE.md](../../../../CLAUDE.md#coding-standards).

Key standards applied in this module:
- **Naming**: `PascalCase` for classes, `camelCase` for methods
- **Virtual destructor**: Always `= default` in base class
- **Memory**: Designed for `std::unique_ptr` ownership

See the [root CLAUDE.md](../../../../CLAUDE.md#coding-standards) for complete details and examples.

---

## Getting Help

### For AI Assistants
1. This document provides complete architectural context for the Agent module
2. Review [Environment/CLAUDE.md](../Environment/CLAUDE.md) for InertialState details
3. Review [msd-sim/CLAUDE.md](../../CLAUDE.md) for overall simulation architecture
4. Check [root CLAUDE.md](../../../../CLAUDE.md) for project-wide conventions

### For Developers
- **Basic agent**: Extend BaseAgent and implement updateState()
- **Platform integration**: Platform owns agents via `std::unique_ptr`
- **State access**: Agent receives current state, returns desired state
