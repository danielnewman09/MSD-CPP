// Ticket: 0035d_friction_hardening_and_validation
// Design: docs/designs/0035d_friction_hardening_and_validation/design.md

#include "M8ScenarioBuilder.hpp"
#include <cmath>
#include <stdexcept>

namespace msd_sim
{

namespace
{
// Standard gravity constant
constexpr double kGravity = 9.81;  // m/s²
}  // namespace

Scenario M8ScenarioBuilder::createStaticFrictionIncline(
    double slopeAngle,
    double frictionCoeff)
{
  // Validate inputs
  if (slopeAngle < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createStaticFrictionIncline: slopeAngle must be "
        "non-negative"};
  }
  if (frictionCoeff < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createStaticFrictionIncline: frictionCoeff must "
        "be non-negative"};
  }

  // M8 Example 1: 10kg block on 20° incline, μ=0.5
  // tan(20°) ≈ 0.364 < 0.5, so block stays stationary
  const double mass = 10.0;  // kg
  const double cosTheta = std::cos(slopeAngle);
  const double sinTheta = std::sin(slopeAngle);

  // Normal force: N = m·g·cos(θ)
  const double normalForce = mass * kGravity * cosTheta;

  // Tangential component of gravity: F_tangent = m·g·sin(θ)
  const double frictionForce = mass * kGravity * sinTheta;

  // Maximum static friction: F_max = μ·N
  const double maxStaticFriction = frictionCoeff * normalForce;

  // Verify static friction can hold (F_tangent < F_max)
  if (frictionForce >= maxStaticFriction)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createStaticFrictionIncline: slopeAngle too "
        "steep for static friction (tan(angle) must be < mu)"};
  }

  Scenario scenario;

  // Object properties
  scenario.mass = mass;
  scenario.frictionCoefficient = frictionCoeff;
  scenario.restitution = 0.0;  // No bouncing
  scenario.cubeSize = 1.0;     // m

  // Initial conditions (block at rest on incline)
  scenario.initialPosition = Coordinate{0.0, 0.0, 1.0};
  scenario.initialVelocity = Coordinate{0.0, 0.0, 0.0};
  scenario.floorPosition = Coordinate{0.0, 0.0, 0.0};

  // External forces
  scenario.slopeAngleRadians = slopeAngle;
  scenario.gravity = Coordinate{0.0, 0.0, -kGravity};

  // Expected results (static equilibrium)
  scenario.expectedNormalForce = normalForce;
  scenario.expectedFrictionForce = frictionForce;
  scenario.expectedVelocity = Coordinate{0.0, 0.0, 0.0};  // No motion
  scenario.expectedAcceleration = 0.0;  // No acceleration
  scenario.expectedStoppingDistance =
      0.0;  // Already stopped
  scenario.expectedAngularVelocity = 0.0;  // No rotation

  // Tolerances
  scenario.forceTolerance = 0.1;             // N
  scenario.velocityTolerance = 1e-6;         // m/s (zero tolerance)
  scenario.accelerationTolerance = 1e-6;     // m/s² (zero tolerance)
  scenario.angularVelocityTolerance = 1e-6;  // rad/s (zero tolerance)

  // Simulation parameters
  scenario.dt = 0.001;    // s
  scenario.numSteps = 1000;  // 1 second simulation

  return scenario;
}

Scenario M8ScenarioBuilder::createKineticFrictionIncline(
    double slopeAngle,
    double frictionCoeff)
{
  // Validate inputs
  if (slopeAngle < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createKineticFrictionIncline: slopeAngle must be "
        "non-negative"};
  }
  if (frictionCoeff < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createKineticFrictionIncline: frictionCoeff must "
        "be non-negative"};
  }

  // M8 Example 2: 10kg block on 45° incline, μ=0.3
  // tan(45°) = 1.0 > 0.3, so block slides
  const double mass = 10.0;  // kg
  const double cosTheta = std::cos(slopeAngle);
  const double sinTheta = std::sin(slopeAngle);

  // Verify kinetic friction regime (tan(θ) > μ)
  if (std::tan(slopeAngle) <= frictionCoeff)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createKineticFrictionIncline: slopeAngle too "
        "shallow for kinetic friction (tan(angle) must be > mu)"};
  }

  // Normal force: N = m·g·cos(θ)
  const double normalForce = mass * kGravity * cosTheta;

  // Kinetic friction force: F_friction = μ·N
  const double frictionForce = frictionCoeff * normalForce;

  // Net acceleration down the incline: a = g(sin(θ) - μ·cos(θ))
  const double acceleration = kGravity * (sinTheta - frictionCoeff * cosTheta);

  Scenario scenario;

  // Object properties
  scenario.mass = mass;
  scenario.frictionCoefficient = frictionCoeff;
  scenario.restitution = 0.0;  // No bouncing
  scenario.cubeSize = 1.0;     // m

  // Initial conditions (block at rest on incline)
  scenario.initialPosition = Coordinate{0.0, 0.0, 1.0};
  scenario.initialVelocity = Coordinate{0.0, 0.0, 0.0};
  scenario.floorPosition = Coordinate{0.0, 0.0, 0.0};

  // External forces
  scenario.slopeAngleRadians = slopeAngle;
  scenario.gravity = Coordinate{0.0, 0.0, -kGravity};

  // Expected results (kinetic sliding)
  scenario.expectedNormalForce = normalForce;
  scenario.expectedFrictionForce = frictionForce;
  scenario.expectedAcceleration = acceleration;
  // Velocity after 1 second: v = a·t
  scenario.expectedVelocity =
      Coordinate{acceleration, 0.0, 0.0};  // Down the incline (X-axis)
  scenario.expectedStoppingDistance =
      std::numeric_limits<double>::quiet_NaN();  // Not applicable (continuous
                                                  // sliding)
  scenario.expectedAngularVelocity = 0.0;  // No rotation (sliding, not rolling)

  // Tolerances
  scenario.forceTolerance = 0.1;              // N
  scenario.velocityTolerance = 0.05;          // 5% relative
  scenario.accelerationTolerance = 0.05;      // 5% relative
  scenario.angularVelocityTolerance = 1e-6;   // rad/s (zero tolerance)

  // Simulation parameters
  scenario.dt = 0.001;    // s
  scenario.numSteps = 1000;  // 1 second simulation

  return scenario;
}

Scenario M8ScenarioBuilder::createSlidingDeceleration(
    double initialVelocity,
    double frictionCoeff)
{
  // Validate inputs
  if (initialVelocity < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createSlidingDeceleration: initialVelocity must "
        "be non-negative"};
  }
  if (frictionCoeff < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createSlidingDeceleration: frictionCoeff must be "
        "non-negative"};
  }

  // M8 Example 3: 10kg block sliding at 5 m/s on flat surface, μ=0.4
  const double mass = 10.0;  // kg

  // Normal force: N = m·g (flat surface)
  const double normalForce = mass * kGravity;

  // Friction force: F = μ·N
  const double frictionForce = frictionCoeff * normalForce;

  // Deceleration: a = -μ·g (negative because opposing motion)
  const double deceleration = frictionCoeff * kGravity;

  // Stopping distance: d = v₀²/(2·μ·g)
  const double stoppingDistance =
      (initialVelocity * initialVelocity) / (2.0 * frictionCoeff * kGravity);

  // Time to stop: t = v₀/(μ·g)
  const double stoppingTime = initialVelocity / (frictionCoeff * kGravity);

  Scenario scenario;

  // Object properties
  scenario.mass = mass;
  scenario.frictionCoefficient = frictionCoeff;
  scenario.restitution = 0.0;  // No bouncing
  scenario.cubeSize = 1.0;     // m

  // Initial conditions (block sliding on flat surface)
  scenario.initialPosition = Coordinate{0.0, 0.0, 1.0};
  scenario.initialVelocity = Coordinate{initialVelocity, 0.0, 0.0};
  scenario.floorPosition = Coordinate{0.0, 0.0, 0.0};

  // External forces (flat surface, no slope)
  scenario.slopeAngleRadians = 0.0;
  scenario.gravity = Coordinate{0.0, 0.0, -kGravity};

  // Expected results (deceleration to stop)
  scenario.expectedNormalForce = normalForce;
  scenario.expectedFrictionForce = frictionForce;
  scenario.expectedAcceleration = -deceleration;  // Negative (decelerating)
  scenario.expectedVelocity = Coordinate{0.0, 0.0, 0.0};  // Stops eventually
  scenario.expectedStoppingDistance = stoppingDistance;
  scenario.expectedAngularVelocity = 0.0;  // No rotation (sliding, not rolling)

  // Tolerances
  scenario.forceTolerance = 0.1;             // N
  scenario.velocityTolerance = 1e-6;         // m/s (zero at end)
  scenario.accelerationTolerance = 0.05;     // 5% relative
  scenario.angularVelocityTolerance = 1e-6;  // rad/s (zero tolerance)

  // Simulation parameters (run until block stops)
  scenario.dt = 0.001;  // s
  // Simulate for 2x stopping time to verify it stays stopped
  scenario.numSteps = static_cast<int>(2.0 * stoppingTime / scenario.dt);

  return scenario;
}

Scenario M8ScenarioBuilder::createGlancingCollision(
    double impactSpeed,
    double frictionCoeff)
{
  // Validate inputs
  if (impactSpeed < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createGlancingCollision: impactSpeed must be "
        "non-negative"};
  }
  if (frictionCoeff < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createGlancingCollision: frictionCoeff must be "
        "non-negative"};
  }

  // M8 Example 4: 1kg cube at 10 m/s hits stationary cube off-center
  // μ=0.8, cube size 0.2m
  const double mass = 1.0;       // kg
  const double cubeSize = 0.2;   // m

  // Off-center impact at edge (lever arm = L/2)
  const double leverArm = cubeSize / 2.0;

  // Tangential impulse at contact point (simplified model)
  // J_t = μ·J_n (friction impulse proportional to normal impulse)
  // For head-on collision: J_n ≈ m·v
  const double normalImpulse = mass * impactSpeed;
  const double tangentialImpulse = frictionCoeff * normalImpulse;

  // Moment of inertia for cube about centroid: I = (1/6)·m·L²
  const double inertia = (1.0 / 6.0) * mass * cubeSize * cubeSize;

  // Angular impulse: L = r × J_t = leverArm · tangentialImpulse
  const double angularImpulse = leverArm * tangentialImpulse;

  // Angular velocity: ω = L/I
  const double angularVelocity = angularImpulse / inertia;

  Scenario scenario;

  // Object properties
  scenario.mass = mass;
  scenario.frictionCoefficient = frictionCoeff;
  scenario.restitution = 0.0;  // Inelastic collision
  scenario.cubeSize = cubeSize;

  // Initial conditions (moving cube about to impact)
  scenario.initialPosition = Coordinate{0.0, 0.0, 1.0};
  scenario.initialVelocity = Coordinate{impactSpeed, 0.0, 0.0};
  scenario.floorPosition = Coordinate{0.0, 0.0, 0.0};

  // External forces
  scenario.slopeAngleRadians = 0.0;
  scenario.gravity = Coordinate{0.0, 0.0, -kGravity};

  // Expected results (spin after collision)
  scenario.expectedNormalForce =
      std::numeric_limits<double>::quiet_NaN();  // Impulse-based, not force
  scenario.expectedFrictionForce =
      std::numeric_limits<double>::quiet_NaN();  // Impulse-based, not force
  scenario.expectedAcceleration =
      std::numeric_limits<double>::quiet_NaN();  // Not applicable for impulsive
                                                  // collision
  scenario.expectedVelocity =
      Coordinate{0.0, 0.0, 0.0};  // Approximately stops (inelastic)
  scenario.expectedStoppingDistance =
      std::numeric_limits<double>::quiet_NaN();  // Not applicable
  scenario.expectedAngularVelocity = angularVelocity;

  // Tolerances
  scenario.forceTolerance = 0.1;              // N (not used for impulse test)
  scenario.velocityTolerance = 0.05;          // 5% relative
  scenario.accelerationTolerance = 0.05;      // 5% relative
  scenario.angularVelocityTolerance = 0.5;    // rad/s (absolute)

  // Simulation parameters (short duration for impulsive collision)
  scenario.dt = 0.0001;  // s (smaller timestep for collision)
  scenario.numSteps = 100;  // 0.01 second simulation

  return scenario;
}

Scenario M8ScenarioBuilder::createConeSaturation(
    double mass,
    double frictionCoeff)
{
  // Validate inputs
  if (mass <= 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createConeSaturation: mass must be positive"};
  }
  if (frictionCoeff < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createConeSaturation: frictionCoeff must be "
        "non-negative"};
  }

  // M8 Example 5: 10kg block on flat surface, μ=0.3
  // Friction cone saturates when applied tangential force exceeds μ·N

  // Normal force: N = m·g
  const double normalForce = mass * kGravity;

  // Friction cone saturation threshold: F_max = μ·N
  const double saturationForce = frictionCoeff * normalForce;

  Scenario scenario;

  // Object properties
  scenario.mass = mass;
  scenario.frictionCoefficient = frictionCoeff;
  scenario.restitution = 0.0;  // No bouncing
  scenario.cubeSize = 1.0;     // m

  // Initial conditions (block at rest on flat surface)
  scenario.initialPosition = Coordinate{0.0, 0.0, 1.0};
  scenario.initialVelocity = Coordinate{0.0, 0.0, 0.0};
  scenario.floorPosition = Coordinate{0.0, 0.0, 0.0};

  // External forces
  scenario.slopeAngleRadians = 0.0;
  scenario.gravity = Coordinate{0.0, 0.0, -kGravity};

  // Expected results (stick-to-slip transition)
  scenario.expectedNormalForce = normalForce;
  scenario.expectedFrictionForce =
      saturationForce;  // Transition force threshold
  scenario.expectedAcceleration = 0.0;  // At threshold (about to slip)
  scenario.expectedVelocity = Coordinate{0.0, 0.0, 0.0};  // Still stationary
  scenario.expectedStoppingDistance =
      std::numeric_limits<double>::quiet_NaN();  // Not applicable
  scenario.expectedAngularVelocity = 0.0;  // No rotation

  // Tolerances
  scenario.forceTolerance = 0.1;             // N (transition detection)
  scenario.velocityTolerance = 1e-6;         // m/s (zero tolerance)
  scenario.accelerationTolerance = 1e-6;     // m/s² (zero tolerance)
  scenario.angularVelocityTolerance = 1e-6;  // rad/s (zero tolerance)

  // Simulation parameters
  scenario.dt = 0.001;    // s
  scenario.numSteps = 1000;  // 1 second simulation

  return scenario;
}

Scenario M8ScenarioBuilder::createTwoBodyFriction(
    double appliedForce,
    double frictionCoeff)
{
  // Validate inputs
  if (appliedForce < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createTwoBodyFriction: appliedForce must be "
        "non-negative"};
  }
  if (frictionCoeff < 0.0)
  {
    throw std::invalid_argument{
        "M8ScenarioBuilder::createTwoBodyFriction: frictionCoeff must be "
        "non-negative"};
  }

  // M8 Example 6: Two 10kg blocks stacked
  // Bottom on frictionless surface, applied force on top block
  // μ=0.5 between blocks
  const double mass = 10.0;  // kg (each block)

  // Normal force between blocks: N = m_top·g
  const double normalForce = mass * kGravity;

  // Maximum friction force between blocks: F_max = μ·N
  const double maxFrictionForce = frictionCoeff * normalForce;

  // Newton's third law: friction on top = -friction on bottom
  // If applied force < F_max: blocks move together, friction = applied force
  // If applied force > F_max: blocks slip, friction = F_max
  const double frictionForce = std::min(appliedForce, maxFrictionForce);

  Scenario scenario;

  // Object properties (top block only for this scenario)
  scenario.mass = mass;
  scenario.frictionCoefficient = frictionCoeff;
  scenario.restitution = 0.0;  // No bouncing
  scenario.cubeSize = 1.0;     // m

  // Initial conditions (blocks at rest)
  scenario.initialPosition = Coordinate{0.0, 0.0, 2.0};  // Top block
  scenario.initialVelocity = Coordinate{0.0, 0.0, 0.0};
  scenario.floorPosition = Coordinate{0.0, 0.0, 0.0};

  // External forces
  scenario.slopeAngleRadians = 0.0;
  scenario.gravity = Coordinate{0.0, 0.0, -kGravity};

  // Expected results (Newton's third law verification)
  scenario.expectedNormalForce = normalForce;
  scenario.expectedFrictionForce = frictionForce;
  scenario.expectedAcceleration =
      std::numeric_limits<double>::quiet_NaN();  // Depends on applied force
  scenario.expectedVelocity = Coordinate{0.0, 0.0, 0.0};  // Initially at rest
  scenario.expectedStoppingDistance =
      std::numeric_limits<double>::quiet_NaN();  // Not applicable
  scenario.expectedAngularVelocity = 0.0;  // No rotation

  // Tolerances
  scenario.forceTolerance = 0.1;             // N (action-reaction balance)
  scenario.velocityTolerance = 0.05;         // 5% relative
  scenario.accelerationTolerance = 0.05;     // 5% relative
  scenario.angularVelocityTolerance = 1e-6;  // rad/s (zero tolerance)

  // Simulation parameters
  scenario.dt = 0.001;    // s
  scenario.numSteps = 1000;  // 1 second simulation

  return scenario;
}

}  // namespace msd_sim
