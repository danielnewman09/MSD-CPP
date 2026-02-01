# M8. Numerical Examples with GTest Mappings

> **Parent**: [math-formulation.md](math-formulation.md)
> **Depends on**: [M1 (Tangent Basis)](M1-tangent-basis.md), [M2 (Friction Jacobian)](M2-friction-jacobian.md), [M3 (Coulomb Cone)](M3-coulomb-cone.md), [M4 (Complementarity)](M4-complementarity.md), [M5 (Solver Extension)](M5-solver-extension.md), [M7 (Numerical Stability)](M7-numerical-stability.md)
> **Required by**: Implementation phase (GTest templates)

---

**Objective**: Provide six concrete worked examples with hand-verified results and GTest templates.

---

## Example 1: Nominal Case — Block on Inclined Plane (Static Friction)

**Scenario**: 10 kg block rests on a 20° inclined plane with coefficient of friction $\mu = 0.6$. Verify that the block remains stationary (static friction balances gravity component).

**Inputs**:
```
mass m = 10.0 kg
gravity g = 9.81 m/s²
slope angle θ = 20° = 0.349 rad
friction coefficient μ = 0.6
initial velocity v₀ = 0 m/s (at rest)
```

**Hand Computation**:

Step 1: Resolve gravity into normal and tangential components.
$$
F_{\text{grav}} = m g = 10 \times 9.81 = 98.1 \text{ N (downward)}
$$

Normal component (perpendicular to slope):
$$
F_n = F_{\text{grav}} \cos\theta = 98.1 \times \cos(20°) = 98.1 \times 0.9397 = 92.18 \text{ N}
$$

Tangential component (down the slope):
$$
F_t = F_{\text{grav}} \sin\theta = 98.1 \times \sin(20°) = 98.1 \times 0.3420 = 33.55 \text{ N}
$$

Step 2: Check friction angle.

Friction angle $\phi_f = \arctan(\mu) = \arctan(0.6) = 30.96°$

Since $\theta = 20° < \phi_f = 30.96°$, static friction can balance the tangential force (block does not slide).

Step 3: Compute constraint forces.

Normal constraint force:
$$
\lambda_n = F_n = 92.18 \text{ N}
$$

Friction force (opposes tangential gravity component, prevents sliding):
$$
\lambda_t = F_t = 33.55 \text{ N}
$$

Step 4: Verify friction cone constraint.

Maximum static friction:
$$
\mu \lambda_n = 0.6 \times 92.18 = 55.31 \text{ N}
$$

Since $\lambda_t = 33.55 \text{ N} < 55.31 \text{ N}$, friction force lies strictly inside the friction cone (stick regime).

Step 5: Verify velocity.

Tangential velocity:
$$
v_t = 0 \text{ m/s (block is stationary)}
$$

**Expected Output**:
```
normal_force λₙ = 92.18 N
friction_force λₜ = 33.55 N (up the slope, opposes gravity)
tangential_velocity vₜ = 0.0 m/s
regime = "stick" (static friction, no sliding)
friction_cone_slack = 55.31 - 33.55 = 21.76 N (force is interior to cone)
```

**GTest Template**:
```cpp
TEST(FrictionConstraint, NominalCase_BlockOnInclinedPlaneStaticFriction) {
  // Inputs from math formulation Example 1
  const double mass{10.0};  // kg
  const double gravity{9.81};  // m/s²
  const double slopeAngle{0.349};  // 20° in radians
  const double mu{0.6};
  const Coordinate initialVelocity{0.0, 0.0, 0.0};  // At rest

  // Set up inclined plane contact
  const Coordinate normal{0.0, std::sin(slopeAngle), std::cos(slopeAngle)};  // Normal perpendicular to slope
  const Coordinate gravityForce{0.0, 0.0, -mass * gravity};  // Downward

  // Create contact constraint with friction
  ContactConstraintFactory factory;
  auto constraint = factory.createWithFriction(
      contactPoint, normal, bodyA, bodyB, mu, restitution);

  // Solve constraint system
  ConstraintSolver solver;
  auto result = solver.solveWithContacts({constraint.get()}, states, masses, inverseInertias, dt);

  // Expected normal force (perpendicular to slope component of gravity)
  const double expectedNormalForce{mass * gravity * std::cos(slopeAngle)};
  const double expectedFrictionForce{mass * gravity * std::sin(slopeAngle)};

  // Tolerances
  constexpr double kForceTolerance = 0.1;  // N
  constexpr double kVelocityTolerance = 1e-6;  // m/s

  // Verify normal force
  EXPECT_NEAR(result.normalForce, expectedNormalForce, kForceTolerance);

  // Verify friction force
  EXPECT_NEAR(result.frictionForce.norm(), expectedFrictionForce, kForceTolerance);

  // Verify velocity is zero (static friction, no sliding)
  EXPECT_NEAR(result.tangentialVelocity.norm(), 0.0, kVelocityTolerance);

  // Verify friction is interior to cone
  const double frictionConeLimit = mu * expectedNormalForce;
  EXPECT_LT(expectedFrictionForce, frictionConeLimit);
}
```

---

## Example 2: Edge Case — Block on Inclined Plane (Kinetic Friction)

**Scenario**: 10 kg block on a 45° inclined plane with coefficient of friction $\mu = 0.3$. Verify that the block accelerates down the slope (kinetic friction insufficient to prevent sliding).

**Inputs**:
```
mass m = 10.0 kg
gravity g = 9.81 m/s²
slope angle θ = 45° = 0.785 rad
friction coefficient μ = 0.3
initial velocity v₀ = 0 m/s (released from rest)
```

**Hand Computation**:

Step 1: Resolve gravity components.
$$
F_n = m g \cos(45°) = 10 \times 9.81 \times 0.7071 = 69.36 \text{ N}
$$
$$
F_t = m g \sin(45°) = 10 \times 9.81 \times 0.7071 = 69.36 \text{ N}
$$

Step 2: Check friction angle.
$$
\phi_f = \arctan(0.3) = 16.70°
$$

Since $\theta = 45° > \phi_f = 16.70°$, static friction cannot hold the block (block will slide).

Step 3: Compute constraint forces (slip regime).

Normal force:
$$
\lambda_n = F_n = 69.36 \text{ N}
$$

Friction force (saturates cone, opposes motion):
$$
\lambda_t = \mu \lambda_n = 0.3 \times 69.36 = 20.81 \text{ N (up the slope)}
$$

Step 4: Net tangential force.
$$
F_{\text{net},t} = F_t - \lambda_t = 69.36 - 20.81 = 48.55 \text{ N (down the slope)}
$$

Step 5: Tangential acceleration.
$$
a_t = \frac{F_{\text{net},t}}{m} = \frac{48.55}{10} = 4.855 \text{ m/s}^2
$$

Alternatively, using the formula $a = g(\sin\theta - \mu \cos\theta)$:
$$
a_t = 9.81 \times (\sin(45°) - 0.3 \times \cos(45°)) = 9.81 \times (0.7071 - 0.2121) = 9.81 \times 0.4950 = 4.856 \text{ m/s}^2
$$

Step 6: Verify friction cone saturation.
$$
\|\boldsymbol{\lambda}_t\| = 20.81 \text{ N} = \mu \lambda_n = 20.81 \text{ N} \quad \checkmark
$$

**Expected Output**:
```
normal_force λₙ = 69.36 N
friction_force λₜ = 20.81 N (up the slope, at cone boundary)
tangential_acceleration aₜ = 4.856 m/s²
regime = "slip" (kinetic friction, block sliding)
friction_cone_slack = 0.0 N (force saturates cone)
```

**GTest Template**:
```cpp
TEST(FrictionConstraint, EdgeCase_BlockOnInclinedPlaneKineticFriction) {
  // Inputs from math formulation Example 2
  const double mass{10.0};  // kg
  const double gravity{9.81};  // m/s²
  const double slopeAngle{0.785};  // 45° in radians
  const double mu{0.3};

  // Expected values
  const double expectedNormalForce{mass * gravity * std::cos(slopeAngle)};  // 69.36 N
  const double expectedFrictionForce{mu * expectedNormalForce};  // 20.81 N
  const double expectedAcceleration{gravity * (std::sin(slopeAngle) - mu * std::cos(slopeAngle))};  // 4.856 m/s²

  // Tolerances
  constexpr double kForceTolerance = 0.1;  // N
  constexpr double kAccelTolerance = 0.01;  // m/s²
  constexpr double kConeTolerance = 1e-4;  // N (friction should saturate cone)

  // Solve (implementation details omitted)
  // ...

  // Verify normal force
  EXPECT_NEAR(result.normalForce, expectedNormalForce, kForceTolerance);

  // Verify friction force magnitude
  EXPECT_NEAR(result.frictionForce.norm(), expectedFrictionForce, kForceTolerance);

  // Verify acceleration (block slides down slope)
  EXPECT_NEAR(result.tangentialAcceleration, expectedAcceleration, kAccelTolerance);

  // Verify friction saturates cone (slip regime)
  const double frictionConeLimit = mu * expectedNormalForce;
  EXPECT_NEAR(result.frictionForce.norm(), frictionConeLimit, kConeTolerance);
}
```

---

## Example 3: Nominal Case — Sliding Deceleration on Flat Surface

**Scenario**: 5 kg block slides on a flat horizontal surface with initial velocity $v_0 = 10$ m/s and coefficient of friction $\mu = 0.5$. Verify that the block decelerates at $a = -\mu g$ until it stops.

**Inputs**:
```
mass m = 5.0 kg
initial_velocity v₀ = 10.0 m/s (horizontal)
friction coefficient μ = 0.5
gravity g = 9.81 m/s²
```

**Hand Computation**:

Step 1: Normal force.

On flat surface, normal force equals weight:
$$
\lambda_n = m g = 5.0 \times 9.81 = 49.05 \text{ N}
$$

Step 2: Friction force.

Friction opposes motion (velocity is positive, friction is negative):
$$
\lambda_t = -\mu \lambda_n = -0.5 \times 49.05 = -24.53 \text{ N}
$$

(Negative sign indicates force opposes motion direction.)

Step 3: Deceleration.
$$
a = \frac{\lambda_t}{m} = \frac{-24.53}{5.0} = -4.905 \text{ m/s}^2
$$

Alternatively:
$$
a = -\mu g = -0.5 \times 9.81 = -4.905 \text{ m/s}^2
$$

Step 4: Stopping distance.

Using $v^2 = v_0^2 + 2 a s$ with $v = 0$:
$$
s = \frac{-v_0^2}{2a} = \frac{-(10)^2}{2 \times (-4.905)} = \frac{-100}{-9.81} = 10.19 \text{ m}
$$

Step 5: Stopping time.

Using $v = v_0 + at$ with $v = 0$:
$$
t = \frac{-v_0}{a} = \frac{-10}{-4.905} = 2.039 \text{ s}
$$

**Expected Output**:
```
normal_force λₙ = 49.05 N
friction_force λₜ = -24.53 N (opposes velocity)
deceleration a = -4.905 m/s²
stopping_distance s = 10.19 m
stopping_time t = 2.039 s
```

**GTest Template**:
```cpp
TEST(FrictionConstraint, NominalCase_SlidingDecelerationFlatSurface) {
  // Inputs from math formulation Example 3
  const double mass{5.0};  // kg
  const double initialVelocity{10.0};  // m/s
  const double mu{0.5};
  const double gravity{9.81};  // m/s²
  const double dt{0.01};  // 10 ms timestep

  // Expected values
  const double expectedDeceleration{-mu * gravity};  // -4.905 m/s²
  const double expectedStoppingTime{-initialVelocity / expectedDeceleration};  // 2.039 s
  const double expectedStoppingDistance{-initialVelocity * initialVelocity / (2 * expectedDeceleration)};  // 10.19 m

  // Simulate until block stops
  double velocity = initialVelocity;
  double position = 0.0;
  double time = 0.0;

  while (velocity > 1e-6) {  // Stop when velocity near zero
    // Apply friction constraint (implementation details omitted)
    // ...

    velocity += expectedDeceleration * dt;
    position += velocity * dt;
    time += dt;
  }

  // Tolerances
  constexpr double kTimeTolerance = 0.05;  // s (5% error acceptable for discrete simulation)
  constexpr double kDistanceTolerance = 0.1;  // m

  // Verify stopping time
  EXPECT_NEAR(time, expectedStoppingTime, kTimeTolerance);

  // Verify stopping distance
  EXPECT_NEAR(position, expectedStoppingDistance, kDistanceTolerance);
}
```

---

## Example 4: Degenerate Case — Glancing Collision with Spin

**Scenario**: 2 kg sphere (radius 0.1 m) collides with a stationary 10 kg block at a $30°$ glancing angle with impact velocity $v = 5$ m/s. Coefficient of friction $\mu = 0.4$. Verify that tangential impulse generates angular velocity (spin).

**Inputs**:
```
sphere mass m_A = 2.0 kg
sphere radius r = 0.1 m
sphere moment of inertia I = (2/5) m r² = 0.008 kg·m²
block mass m_B = 10.0 kg
impact velocity v = 5.0 m/s
impact angle θ = 30° (relative to normal)
friction coefficient μ = 0.4
```

**Hand Computation**:

Step 1: Decompose impact velocity.

Normal component:
$$
v_n = v \cos(30°) = 5.0 \times 0.8660 = 4.330 \text{ m/s}
$$

Tangential component:
$$
v_t = v \sin(30°) = 5.0 \times 0.5 = 2.5 \text{ m/s}
$$

Step 2: Compute effective mass in normal direction.
$$
m_{\text{eff},n} = \frac{m_A m_B}{m_A + m_B} = \frac{2.0 \times 10.0}{12.0} = 1.667 \text{ kg}
$$

Step 3: Normal impulse (assuming elastic collision, $e = 1$).
$$
J_n = m_{\text{eff},n} (1 + e) v_n = 1.667 \times 2 \times 4.330 = 14.44 \text{ N·s}
$$

Step 4: Maximum friction impulse.
$$
J_{t,\max} = \mu J_n = 0.4 \times 14.44 = 5.776 \text{ N·s}
$$

Step 5: Tangential impulse (check if sliding occurs).

Effective mass in tangential direction (including rotational inertia):

For a sphere hitting at edge, contact point is at distance $r$ from center of mass. Effective mass including rotation:
$$
m_{\text{eff},t} = \left(\frac{1}{m_A} + \frac{r^2}{I_A} + \frac{1}{m_B}\right)^{-1} = \left(\frac{1}{2.0} + \frac{0.01}{0.008} + \frac{1}{10.0}\right)^{-1}
$$
$$
= \left(0.5 + 1.25 + 0.1\right)^{-1} = (1.85)^{-1} = 0.541 \text{ kg}
$$

Tangential impulse required to stop sliding:
$$
J_t = m_{\text{eff},t} v_t = 0.541 \times 2.5 = 1.352 \text{ N·s}
$$

Since $J_t = 1.352 < J_{t,\max} = 5.776$, friction is sufficient to eliminate tangential velocity (stick regime).

Step 6: Angular impulse.

Torque arm: $r = 0.1$ m (contact point at edge of sphere)

Angular impulse:
$$
L = r J_t = 0.1 \times 1.352 = 0.1352 \text{ kg·m²/s}
$$

Step 7: Resulting angular velocity.
$$
\omega = \frac{L}{I} = \frac{0.1352}{0.008} = 16.90 \text{ rad/s}
$$

**Why This Is Degenerate**: Glancing collision involves both normal and tangential impulses, with rotational coupling. Friction must prevent sliding while transferring angular momentum. This tests the correctness of the Jacobian term $\mathbf{r} \times \mathbf{t}$ in converting tangential force to torque.

**Expected Behavior**: Sphere gains angular velocity proportional to tangential impulse and moment arm, demonstrating that friction produces torque for off-center impacts.

**Expected Output**:
```
normal_impulse Jₙ = 14.44 N·s
tangential_impulse Jₜ = 1.352 N·s
angular_impulse L = 0.1352 kg·m²/s
resulting_angular_velocity ω = 16.90 rad/s
regime = "stick" (friction prevents sliding)
```

**GTest Template**:
```cpp
TEST(FrictionConstraint, DegenerateCase_GlancingCollisionWithSpin) {
  // Inputs from math formulation Example 4
  const double sphereMass{2.0};  // kg
  const double sphereRadius{0.1};  // m
  const double sphereInertia{0.4 * sphereMass * sphereRadius * sphereRadius};  // (2/5) m r²
  const double blockMass{10.0};  // kg
  const double impactVelocity{5.0};  // m/s
  const double impactAngle{30.0 * M_PI / 180.0};  // radians
  const double mu{0.4};
  const double restitution{1.0};  // Elastic collision

  // Decompose velocity
  const double normalVelocity{impactVelocity * std::cos(impactAngle)};
  const double tangentialVelocity{impactVelocity * std::sin(impactAngle)};

  // Expected angular velocity from tangential impulse
  const double effectiveMassNormal{(sphereMass * blockMass) / (sphereMass + blockMass)};
  const double normalImpulse{effectiveMassNormal * (1 + restitution) * normalVelocity};
  const double maxFrictionImpulse{mu * normalImpulse};

  // Effective tangential mass (including rotation)
  const double effectiveMassTangential{1.0 / (
      1.0 / sphereMass +
      (sphereRadius * sphereRadius) / sphereInertia +
      1.0 / blockMass)};
  const double tangentialImpulse{effectiveMassTangential * tangentialVelocity};

  const double angularImpulse{sphereRadius * tangentialImpulse};
  const double expectedAngularVelocity{angularImpulse / sphereInertia};

  // Solve collision with friction (implementation details omitted)
  // ...

  // Tolerances
  constexpr double kAngularVelocityTolerance = 0.5;  // rad/s
  constexpr double kImpulseTolerance = 0.1;  // N·s

  // Verify tangential impulse is within friction cone
  EXPECT_LT(tangentialImpulse, maxFrictionImpulse);

  // Verify angular velocity from friction torque
  EXPECT_NEAR(result.angularVelocity, expectedAngularVelocity, kAngularVelocityTolerance);

  // Verify tangential velocity reduced to zero (stick regime)
  EXPECT_NEAR(result.tangentialVelocity, 0.0, 1e-6);
}
```

---

## Example 5: Edge Case — Friction Cone Saturation (Stick-to-Slip Transition)

**Scenario**: 8 kg block on a surface with $\mu = 0.3$. Applied horizontal force increases from 0 N to 30 N. Verify transition from stick (friction balances applied force) to slip (friction saturates at $\mu N$).

**Inputs**:
```
mass m = 8.0 kg
gravity g = 9.81 m/s²
friction coefficient μ = 0.3
applied force F_applied = [0, 5, 10, 15, 20, 25, 30] N (increasing)
```

**Hand Computation**:

Step 1: Normal force.
$$
\lambda_n = m g = 8.0 \times 9.81 = 78.48 \text{ N}
$$

Step 2: Maximum static friction.
$$
\lambda_{t,\max} = \mu \lambda_n = 0.3 \times 78.48 = 23.54 \text{ N}
$$

Step 3: For each applied force, determine regime.

| $F_{\text{applied}}$ (N) | $F_{\text{applied}} < \lambda_{t,\max}$? | Regime | $\lambda_t$ (N) | $v_t$ (m/s) | Acceleration (m/s²) |
|-----------|-----------|--------|--------------|---------|---------------------|
| 0 | Yes | Stick | 0 | 0 | 0 |
| 5 | Yes | Stick | 5 | 0 | 0 |
| 10 | Yes | Stick | 10 | 0 | 0 |
| 15 | Yes | Stick | 15 | 0 | 0 |
| 20 | Yes | Stick | 20 | 0 | 0 |
| 25 | No | **Slip** | 23.54 | $> 0$ | $(25 - 23.54) / 8 = 0.183$ |
| 30 | No | Slip | 23.54 | $> 0$ | $(30 - 23.54) / 8 = 0.807$ |

**Transition point**: When $F_{\text{applied}}$ exceeds $23.54$ N, the friction force saturates and the block begins to slide.

**Expected Output**:
```
For F_applied = 20 N:
  regime = "stick"
  friction_force λₜ = 20 N (balances applied force)
  velocity vₜ = 0 m/s

For F_applied = 25 N:
  regime = "slip"
  friction_force λₜ = 23.54 N (saturated at μ·N)
  net_force F_net = 25 - 23.54 = 1.46 N
  acceleration a = 1.46 / 8 = 0.183 m/s²
```

**GTest Template**:
```cpp
TEST(FrictionConstraint, EdgeCase_FrictionConeSaturationStickToSlip) {
  // Inputs from math formulation Example 5
  const double mass{8.0};  // kg
  const double gravity{9.81};  // m/s²
  const double mu{0.3};

  const double normalForce{mass * gravity};  // 78.48 N
  const double maxStaticFriction{mu * normalForce};  // 23.54 N

  // Test applied forces below and above friction limit
  const std::vector<double> appliedForces{0, 5, 10, 15, 20, 25, 30};

  for (double F_applied : appliedForces) {
    // Solve constraint system with applied force (implementation omitted)
    // ...

    if (F_applied <= maxStaticFriction) {
      // Expect stick regime
      EXPECT_NEAR(result.frictionForce, F_applied, 1e-6);
      EXPECT_NEAR(result.tangentialVelocity, 0.0, 1e-6);
      EXPECT_EQ(result.regime, "stick");
    } else {
      // Expect slip regime
      EXPECT_NEAR(result.frictionForce, maxStaticFriction, 1e-4);
      EXPECT_GT(result.tangentialVelocity, 0.0);  // Block is sliding
      EXPECT_EQ(result.regime, "slip");

      // Verify net acceleration
      const double expectedAcceleration = (F_applied - maxStaticFriction) / mass;
      EXPECT_NEAR(result.acceleration, expectedAcceleration, 0.01);
    }
  }
}
```

---

## Example 6: Nominal Case — Two-Body Friction (Newton's Third Law)

**Scenario**: Block A (3 kg) sits on top of Block B (5 kg). Horizontal force $F = 10$ N applied to block A. Coefficient of friction between A and B is $\mu = 0.5$. Verify that friction forces obey Newton's third law ($\boldsymbol{\lambda}_{t,A} = -\boldsymbol{\lambda}_{t,B}$) and both blocks accelerate together.

**Inputs**:
```
mass m_A = 3.0 kg (on top)
mass m_B = 5.0 kg (on bottom)
applied force F = 10.0 N (on block A, horizontal)
friction coefficient μ = 0.5
gravity g = 9.81 m/s²
```

**Hand Computation**:

Step 1: Normal force (weight of A on B).
$$
\lambda_n = m_A g = 3.0 \times 9.81 = 29.43 \text{ N}
$$

Step 2: Maximum friction force.
$$
\lambda_{t,\max} = \mu \lambda_n = 0.5 \times 29.43 = 14.72 \text{ N}
$$

Step 3: If blocks move together, system acceleration:
$$
a = \frac{F}{m_A + m_B} = \frac{10.0}{8.0} = 1.25 \text{ m/s}^2
$$

Step 4: Friction force required to accelerate block B at $1.25$ m/s²:
$$
\lambda_t = m_B \times a = 5.0 \times 1.25 = 6.25 \text{ N}
$$

Step 5: Check if friction is sufficient.

Since $\lambda_t = 6.25 \text{ N} < \lambda_{t,\max} = 14.72 \text{ N}$, friction does not slip. Blocks move together.

Step 6: Verify Newton's third law.

Force on block B from block A (friction, forward):
$$
\boldsymbol{\lambda}_{t,B} = +6.25 \text{ N}
$$

Force on block A from block B (friction, backward):
$$
\boldsymbol{\lambda}_{t,A} = -6.25 \text{ N}
$$

Net force on A:
$$
F_{\text{net},A} = 10.0 - 6.25 = 3.75 \text{ N} \Rightarrow a_A = 3.75 / 3.0 = 1.25 \text{ m/s}^2 \quad \checkmark
$$

Net force on B:
$$
F_{\text{net},B} = 6.25 \text{ N} \Rightarrow a_B = 6.25 / 5.0 = 1.25 \text{ m/s}^2 \quad \checkmark
$$

**Expected Output**:
```
normal_force λₙ = 29.43 N
friction_force_on_B λₜ_B = +6.25 N (forward, from A)
friction_force_on_A λₜ_A = -6.25 N (backward, from B)
acceleration_A a_A = 1.25 m/s²
acceleration_B a_B = 1.25 m/s²
regime = "stick" (blocks move together)
Newton's third law: λₜ_A + λₜ_B = 0 ✓
```

**GTest Template**:
```cpp
TEST(FrictionConstraint, NominalCase_TwoBodyFrictionNewtonsThirdLaw) {
  // Inputs from math formulation Example 6
  const double massA{3.0};  // kg (on top)
  const double massB{5.0};  // kg (on bottom)
  const double appliedForce{10.0};  // N (on block A)
  const double mu{0.5};
  const double gravity{9.81};  // m/s²

  // Expected values
  const double normalForce{massA * gravity};  // 29.43 N
  const double systemAcceleration{appliedForce / (massA + massB)};  // 1.25 m/s²
  const double frictionForce{massB * systemAcceleration};  // 6.25 N
  const double maxFriction{mu * normalForce};  // 14.72 N

  // Solve two-body contact constraint with friction
  // ...

  // Tolerances
  constexpr double kForceTolerance = 0.1;  // N
  constexpr double kAccelTolerance = 0.01;  // m/s²

  // Verify friction force
  EXPECT_NEAR(result.frictionForceB, frictionForce, kForceTolerance);
  EXPECT_NEAR(result.frictionForceA, -frictionForce, kForceTolerance);

  // Verify Newton's third law
  EXPECT_NEAR(result.frictionForceA + result.frictionForceB, 0.0, 1e-6);

  // Verify both blocks accelerate together
  EXPECT_NEAR(result.accelerationA, systemAcceleration, kAccelTolerance);
  EXPECT_NEAR(result.accelerationB, systemAcceleration, kAccelTolerance);

  // Verify friction is within cone (stick regime)
  EXPECT_LT(frictionForce, maxFriction);
}
```
