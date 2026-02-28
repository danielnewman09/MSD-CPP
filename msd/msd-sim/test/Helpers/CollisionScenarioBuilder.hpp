// Ticket: 0087_collision_solver_lambda_test_suite
// Design: docs/designs/0087_collision_solver_lambda_test_suite/design.md

#ifndef MSD_SIM_TEST_HELPERS_COLLISION_SCENARIO_BUILDER_HPP
#define MSD_SIM_TEST_HELPERS_COLLISION_SCENARIO_BUILDER_HPP

#include <vector>

#include "msd-sim/src/DataTypes/Coordinate.hpp"
#include "msd-sim/src/Environment/ReferenceFrame.hpp"
#include "msd-sim/test/Helpers/CollisionScenario.hpp"

namespace msd_sim::test
{

/**
 * @brief Static factory methods for common collision scenarios.
 *
 * Eliminates copy-paste boilerplate across the many individual lambda tests by
 * providing pre-configured scenarios for the most frequent test setups. All
 * geometry is hardcoded (unit cube / floor slab) — no database dependency,
 * making scenarios self-contained and fast.
 *
 * Each factory returns CollisionScenario by value. NRVO applies at the call
 * site; if a move is needed, the unique_ptr<CollisionPipeline> member enables
 * it.
 *
 * Geometry conventions:
 *  - Unit cube: 8 vertices at (±0.5, ±0.5, ±0.5) — 1m × 1m × 1m box
 *  - Floor slab: flat box from z = -0.1 to z = 0, spanning ±50 m in XY
 *  - Origin of inertial body is at its centre of mass (cube centre)
 *  - Floor surface is at z = 0; cube resting on floor has centre at z = 0.5
 *
 * @ticket 0087_collision_solver_lambda_test_suite
 */
class CollisionScenarioBuilder
{
public:
  // ===== Geometry helpers =====

  /**
   * @brief Unit cube vertex list: 8 vertices at (±0.5, ±0.5, ±0.5).
   *
   * Matches the "unit_cube" asset in the test database.
   * Self-contained — no file I/O required.
   */
  static std::vector<Coordinate> unitCubePoints()
  {
    return {Coordinate{-0.5, -0.5, -0.5},
            Coordinate{+0.5, -0.5, -0.5},
            Coordinate{+0.5, +0.5, -0.5},
            Coordinate{-0.5, +0.5, -0.5},
            Coordinate{-0.5, -0.5, +0.5},
            Coordinate{+0.5, -0.5, +0.5},
            Coordinate{+0.5, +0.5, +0.5},
            Coordinate{-0.5, +0.5, +0.5}};
  }

  /**
   * @brief Thin floor slab vertex list: flat box spanning ±50 m in XY,
   * from z = -0.1 m to z = 0 m.
   *
   * Surface at z = 0. Placed via ReferenceFrame at origin.
   */
  static std::vector<Coordinate> floorSlabPoints()
  {
    return {Coordinate{-50.0, -50.0, -0.1},
            Coordinate{+50.0, -50.0, -0.1},
            Coordinate{+50.0, +50.0, -0.1},
            Coordinate{-50.0, +50.0, -0.1},
            Coordinate{-50.0, -50.0, +0.0},
            Coordinate{+50.0, -50.0, +0.0},
            Coordinate{+50.0, +50.0, +0.0},
            Coordinate{-50.0, +50.0, +0.0}};
  }

  // ===== Factory methods =====

  /**
   * @brief Cube (unit cube geometry) dropped onto a flat floor at z = 0.
   *
   * The cube centre is placed at (0, 0, height). A velocity of (0, 0,
   * velocityZ) is applied. The floor slab is placed at origin (surface at
   * z = 0). For the cube to be in contact at the start of stepOnce(), use
   * height ≈ 0.5 (cube half-extent) or slightly below for immediate contact.
   *
   * @param height Initial z-position of the cube centre [m]
   * @param velocityZ Initial z-velocity (negative = downward) [m/s]
   * @param velocityX Initial x-velocity (for friction tests) [m/s]
   * @param mass Cube mass [kg]
   * @param restitution Coefficient of restitution [0, 1]
   * @param friction Friction coefficient [0, inf)
   * @param dt Simulation time step [s]
   * @return Configured CollisionScenario (body index 0 = cube)
   */
  static CollisionScenario cubeOnFloor(double height,
                                       double velocityZ,
                                       double velocityX,
                                       double mass,
                                       double restitution,
                                       double friction,
                                       double dt = 0.016)
  {
    CollisionScenario scenario{dt};

    // Cube at given height
    const ReferenceFrame cubeFrame{Coordinate{0.0, 0.0, height}};
    scenario.addInertial(unitCubePoints(),
                         cubeFrame,
                         mass,
                         restitution,
                         friction,
                         Coordinate{velocityX, 0.0, velocityZ});

    // Floor at origin (slab surface at z=0)
    const ReferenceFrame floorFrame{Coordinate{0.0, 0.0, 0.0}};
    scenario.addEnvironment(floorSlabPoints(),
                            floorFrame,
                            restitution,
                            friction);

    return scenario;
  }

  /**
   * @brief Sphere-like object (unit cube approximation) dropped onto a flat
   * floor at z = 0.
   *
   * Identical geometry to cubeOnFloor; the "sphere" terminology is used in
   * Stage B / C tests to indicate a single contact-point intent. The cube's
   * flat bottom face will generate a contact manifold of up to 4 contact
   * points. Tests that need single-contact behaviour should position the cube
   * so that only one corner makes contact (e.g., tilted frame), or should
   * account for the multi-point manifold in tolerance calculations.
   *
   * For Stage B normal-impulse tests, the flat-face contact is preferred
   * because the normal direction is aligned with z, making analytical
   * expected lambdas straightforward to compute.
   *
   * @param height Initial z-position of cube centre [m]
   * @param velocityZ Initial z-velocity (negative = downward) [m/s]
   * @param mass Body mass [kg]
   * @param restitution Coefficient of restitution [0, 1]
   * @param friction Friction coefficient [0, inf)
   * @param dt Simulation time step [s]
   * @return Configured CollisionScenario (body index 0 = cube/sphere)
   */
  static CollisionScenario sphereOnFloor(double height,
                                         double velocityZ,
                                         double mass,
                                         double restitution,
                                         double friction,
                                         double dt = 0.016)
  {
    return cubeOnFloor(height, velocityZ, 0.0, mass, restitution, friction, dt);
  }

  /**
   * @brief Two cubes approaching each other along the X axis (zero-gravity
   * context — gravity is NOT applied here; caller must disable gravity on
   * the containing engine or use CollisionScenario directly without an engine).
   *
   * Cube A is centred at (-separation/2, 0, 0) with x-velocity +velA.
   * Cube B is centred at (+separation/2, 0, 0) with x-velocity +velB
   * (pass a negative value for B moving toward A).
   *
   * For an immediate collision, set separation <= 1.0 (sum of half-extents).
   * For a just-touching scenario, set separation = 1.0 exactly.
   *
   * @param separation Distance between the two cube centres along X [m]
   * @param velA X-velocity of cube A (positive = rightward toward B) [m/s]
   * @param velB X-velocity of cube B (negative = leftward toward A) [m/s]
   * @param massA Mass of cube A [kg]
   * @param massB Mass of cube B [kg]
   * @param restitution Coefficient of restitution [0, 1] (applied to both)
   * @param friction Friction coefficient [0, inf) (applied to both)
   * @param dt Simulation time step [s]
   * @return Configured CollisionScenario (body 0 = cube A, body 1 = cube B)
   */
  static CollisionScenario twoCubes(double separation,
                                    double velA,
                                    double velB,
                                    double massA,
                                    double massB,
                                    double restitution,
                                    double friction,
                                    double dt = 0.016)
  {
    CollisionScenario scenario{dt};

    const double halfSep{separation / 2.0};

    const ReferenceFrame frameA{Coordinate{-halfSep, 0.0, 0.0}};
    scenario.addInertial(unitCubePoints(),
                         frameA,
                         massA,
                         restitution,
                         friction,
                         Coordinate{velA, 0.0, 0.0});

    const ReferenceFrame frameB{Coordinate{+halfSep, 0.0, 0.0}};
    scenario.addInertial(unitCubePoints(),
                         frameB,
                         massB,
                         restitution,
                         friction,
                         Coordinate{velB, 0.0, 0.0});

    return scenario;
  }

  /**
   * @brief Two cubes with a controlled overlap for penetration depth tests.
   *
   * Cube A is centred at (-0.5 + overlap/2, 0, 0) and Cube B at
   * (+0.5 - overlap/2, 0, 0), so their faces penetrate by exactly 'overlap'
   * metres along X.
   *
   * Both cubes start at rest (zero velocity). Primarily used in Stage D
   * position correction tests.
   *
   * @param overlap Penetration depth (positive = overlapping) [m]
   * @param mass Mass of each cube [kg]
   * @param restitution Coefficient of restitution [0, 1]
   * @param friction Friction coefficient [0, inf)
   * @param dt Simulation time step [s]
   * @return Configured CollisionScenario (body 0 = cube A, body 1 = cube B)
   */
  static CollisionScenario overlappingCubes(double overlap,
                                            double mass,
                                            double restitution,
                                            double friction,
                                            double dt = 0.016)
  {
    // Total separation between centres = 1.0 (two half-extents) - overlap
    const double separation{1.0 - overlap};
    return twoCubes(separation,
                    0.0,   // velA (at rest)
                    0.0,   // velB (at rest)
                    mass,
                    mass,
                    restitution,
                    friction,
                    dt);
  }
};

}  // namespace msd_sim::test

#endif  // MSD_SIM_TEST_HELPERS_COLLISION_SCENARIO_BUILDER_HPP
