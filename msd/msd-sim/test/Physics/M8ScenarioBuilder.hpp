// Ticket: 0035d_friction_hardening_and_validation
// Design: docs/designs/0035d_friction_hardening_and_validation/design.md

#ifndef MSD_SIM_TEST_M8_SCENARIO_BUILDER_HPP
#define MSD_SIM_TEST_M8_SCENARIO_BUILDER_HPP

#include <limits>
#include "msd-sim/src/Environment/Coordinate.hpp"

namespace msd_sim
{

/**
 * @brief Test scenario data for M8 numerical validation examples
 *
 * Bundles initial conditions, expected results, and tolerances for each
 * M8 numerical example. Used by FrictionValidationTest to parameterize tests.
 *
 * @ticket 0035d_friction_hardening_and_validation
 */
struct Scenario
{
  // Object properties
  double mass{std::numeric_limits<double>::quiet_NaN()};
  double frictionCoefficient{std::numeric_limits<double>::quiet_NaN()};
  double restitution{std::numeric_limits<double>::quiet_NaN()};

  // Geometry
  double cubeSize{1.0};

  // Initial conditions
  Coordinate initialPosition;
  Coordinate initialVelocity;
  Coordinate floorPosition;

  // External forces / geometry
  double slopeAngleRadians{0.0};  // Inclined plane angle
  Coordinate gravity{0.0, 0.0, -9.81};

  // Expected results
  double expectedNormalForce{std::numeric_limits<double>::quiet_NaN()};
  double expectedFrictionForce{std::numeric_limits<double>::quiet_NaN()};
  Coordinate expectedVelocity;
  double expectedAcceleration{std::numeric_limits<double>::quiet_NaN()};
  double expectedStoppingDistance{std::numeric_limits<double>::quiet_NaN()};
  double expectedAngularVelocity{std::numeric_limits<double>::quiet_NaN()};

  // Tolerances
  double forceTolerance{0.1};
  double velocityTolerance{0.05};  // 5% relative
  double accelerationTolerance{0.05};  // 5% relative
  double angularVelocityTolerance{0.5};  // rad/s

  // Simulation parameters
  double dt{0.001};  // timestep [s]
  int numSteps{1000};  // number of timesteps to simulate
};

/**
 * @brief Construct test scenarios corresponding to M8 numerical examples
 *
 * Static-only utility class. Each factory method returns a Scenario struct
 * configured for one M8 example with hand-computed expected values.
 *
 * Thread safety: Pure static functions, thread-safe
 * Error handling: Throws std::invalid_argument for invalid parameters
 *
 * @ticket 0035d_friction_hardening_and_validation
 */
class M8ScenarioBuilder
{
public:
  /**
   * @brief M8 Example 1: Static friction on inclined plane
   *
   * Block on inclined plane, gravity pulls it down but friction holds.
   * Expected: v_t = 0 (no sliding), λ_t < μ·λ_n (stick regime)
   *
   * @param slopeAngle Incline angle [radians] (must be < atan(mu))
   * @param frictionCoeff Friction coefficient μ
   * @return Scenario with expected static equilibrium
   */
  static Scenario createStaticFrictionIncline(
      double slopeAngle,
      double frictionCoeff);

  /**
   * @brief M8 Example 2: Kinetic friction on inclined plane
   *
   * Block sliding down steep inclined plane with friction.
   * Expected: a = g(sinθ - μ·cosθ)
   *
   * @param slopeAngle Incline angle [radians] (must be > atan(mu))
   * @param frictionCoeff Friction coefficient μ
   * @return Scenario with expected acceleration
   */
  static Scenario createKineticFrictionIncline(
      double slopeAngle,
      double frictionCoeff);

  /**
   * @brief M8 Example 3: Sliding deceleration on flat surface
   *
   * Block sliding on flat surface decelerates due to friction.
   * Expected: stops at distance v₀²/(2μg)
   *
   * @param initialVelocity Initial sliding velocity [m/s]
   * @param frictionCoeff Friction coefficient μ
   * @return Scenario with expected stopping distance
   */
  static Scenario createSlidingDeceleration(
      double initialVelocity,
      double frictionCoeff);

  /**
   * @brief M8 Example 4: Glancing collision with spin
   *
   * Off-center impact produces tangential impulse and angular velocity.
   * Expected: ω ≈ 16.90 rad/s
   *
   * @param impactSpeed Impact speed [m/s]
   * @param frictionCoeff Friction coefficient μ
   * @return Scenario with expected angular velocity
   */
  static Scenario createGlancingCollision(
      double impactSpeed,
      double frictionCoeff);

  /**
   * @brief M8 Example 5: Friction cone saturation (stick-to-slip transition)
   *
   * Increasing applied force until friction cone is saturated.
   * Expected: transition at F = μ·m·g
   *
   * @param mass Object mass [kg]
   * @param frictionCoeff Friction coefficient μ
   * @return Scenario with expected transition force
   */
  static Scenario createConeSaturation(
      double mass,
      double frictionCoeff);

  /**
   * @brief M8 Example 6: Two-body friction (Newton's third law)
   *
   * Two objects in contact with applied force.
   * Expected: λ_t,A = -λ_t,B (action-reaction)
   *
   * @param appliedForce Applied force [N]
   * @param frictionCoeff Friction coefficient μ
   * @return Scenario with expected force balance
   */
  static Scenario createTwoBodyFriction(
      double appliedForce,
      double frictionCoeff);

  // Static-only utility (deleted special members)
  ~M8ScenarioBuilder() = delete;
  M8ScenarioBuilder() = delete;
  M8ScenarioBuilder(const M8ScenarioBuilder&) = delete;
  M8ScenarioBuilder(M8ScenarioBuilder&&) = delete;
  M8ScenarioBuilder& operator=(const M8ScenarioBuilder&) = delete;
  M8ScenarioBuilder& operator=(M8ScenarioBuilder&&) = delete;
};

}  // namespace msd_sim

#endif  // MSD_SIM_TEST_M8_SCENARIO_BUILDER_HPP
