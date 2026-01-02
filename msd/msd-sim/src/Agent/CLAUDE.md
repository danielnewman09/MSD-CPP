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
