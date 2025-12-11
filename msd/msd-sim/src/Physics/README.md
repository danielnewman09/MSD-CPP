# Physics Module

This module provides force-based rigid body dynamics for the msd-sim library.

## Overview

The physics module allows objects to have forces applied at points on their geometric boundary, which creates both linear and angular accelerations based on mass and moment of inertia.

## Components

### Core Classes

1. **Force** (`Force.hpp`)
   - Represents a force vector applied at a specific point
   - Point of application is in the object's local reference frame
   - Can calculate torque about any reference point

2. **ForceAccumulator** (`ForceAccumulator.hpp`)
   - Collects multiple forces acting on a rigid body
   - Computes net force and net torque
   - Efficient batch calculations

3. **RigidBodyProperties** (`RigidBodyProperties.hpp`)
   - Contains mass, inertia tensor, and center of mass
   - Caches inverse values for efficiency
   - Factory methods for common shapes (sphere, box, cylinder)

4. **PhysicsState** (`PhysicsState.hpp`)
   - Extension structure for Platform objects
   - Contains RigidBodyProperties and ForceAccumulator
   - Provides convenience methods for force application

5. **PhysicsUtils** (`PhysicsUtils.hpp`)
   - Utility functions for physics integration
   - State integration methods (Euler, semi-implicit Euler, RK4)
   - Conversion between angular representations

## Integration with Platform

To add physics to a Platform object, you would:

1. Add a `PhysicsState` member to the Platform class
2. Use `applyForce()` methods to accumulate forces during simulation
3. In the `update()` method:
   - Calculate accelerations using `PhysicsState::calculateAccelerations()`
   - Integrate the state using functions from `PhysicsUtils`
   - Clear forces for the next step

### Example Integration Pattern

```cpp
// In Platform.hpp, add:
#include "msd-sim/src/Physics/PhysicsState.hpp"
#include <optional>

class Platform {
  // ... existing members ...
  std::optional<PhysicsState> physics_;  // Optional physics support

public:
  // Enable physics for this platform
  void enablePhysics(const RigidBodyProperties& properties);

  // Apply forces (if physics enabled)
  void applyForce(const Force& force);
  void applyForceAtPoint(const Coordinate& forceVector,
                         const Coordinate& point);
};

// In Platform::update():
void Platform::update(const std::chrono::milliseconds& currTime) {
  if (physics_.has_value()) {
    // Calculate time step
    float dt = (currTime - lastUpdateTime_).count() / 1000.0f;

    // Calculate accelerations from forces
    Coordinate linearAccel, angularAccel;
    physics_->calculateAccelerations(
      physics_->properties.getCenterOfMass(),
      linearAccel,
      angularAccel
    );

    // Update state acceleration fields
    state_.acceleration = linearAccel;
    // (convert angularAccel to EulerAngles for state_.angularAcceleration)

    // Integrate state forward
    physics::integrateStateSemiImplicitEuler(
      state_,
      linearAccel,
      angularAccel,
      dt
    );

    // Clear forces for next step
    physics_->clearForces();
  }

  // ... rest of update logic ...
}
```

## Force Application

Forces are applied at points on the geometric boundary of an object:

```cpp
// Create a force
Force thrustForce(
  Coordinate(0, 0, 100),  // 100N in +Z direction
  Coordinate(0, 0, -1)     // Applied at back of object
);

// Apply to platform
platform.applyForce(thrustForce);

// Or use convenience method
platform.applyForceAtPoint(
  Coordinate(10, 0, 0),   // 10N in +X direction
  Coordinate(0, 0.5, 0)   // Applied at top of object
);
```

## Physics Calculations

### Linear Acceleration
```
F_net = Σ F_i
a = F_net / m
```

### Angular Acceleration
```
τ_net = Σ (r_i × F_i)    where r_i = applicationPoint - centerOfMass
α = I^(-1) * τ_net
```

### State Integration
Multiple integration schemes available:
- **Euler**: Simple first-order, fast but least accurate
- **Semi-implicit Euler**: Better stability, recommended for most cases
- **RK4**: Higher accuracy, more expensive

## Units

All quantities use SI units:
- Mass: kilograms [kg]
- Force: Newtons [N]
- Torque: Newton-meters [N⋅m]
- Linear acceleration: meters per second squared [m/s²]
- Angular acceleration: radians per second squared [rad/s²]
- Position: meters [m]
- Inertia: kilogram-meters squared [kg⋅m²]

## Design Decisions

1. **Forces in local frame**: Application points are specified in the object's local reference frame, making it easier to apply forces relative to the object's geometry.

2. **Homogeneous density assumption**: Center of mass is computed from geometry assuming uniform density.

3. **Cached inverse values**: RigidBodyProperties caches inverse mass and inverse inertia tensor to avoid repeated matrix inversions.

4. **Force accumulation**: Forces are collected throughout a time step and applied all at once, which is more physically accurate than applying them sequentially.

5. **Optional physics**: Platform objects can opt-in to physics using `std::optional<PhysicsState>`, maintaining backward compatibility.

## Next Steps

To complete the implementation, you would need to:

1. Implement the `.cpp` files with actual math calculations
2. Extend Platform class to integrate PhysicsState
3. Add geometry support for boundary validation
4. Create test cases for each component
5. Consider adding specialized force types (gravity, drag, thrust, etc.)
