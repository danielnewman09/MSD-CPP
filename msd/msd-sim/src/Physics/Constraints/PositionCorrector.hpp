// Ticket: 0040b_split_impulse_position_correction
// Design: docs/designs/0040b-split-impulse-position-correction/design.md

#ifndef MSD_SIM_PHYSICS_POSITION_CORRECTOR_HPP
#define MSD_SIM_PHYSICS_POSITION_CORRECTOR_HPP

#include <vector>

#include <Eigen/Dense>

#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

/// @brief Position-level contact correction using pseudo-velocities
///
/// Corrects penetration depth by computing position-level impulses that
/// modify body positions and orientations without affecting real velocities.
/// This is the position correction pass of the split impulse approach.
///
/// Algorithm:
/// 1. For each contact, compute position-level bias:
///    b_pos = (beta/dt) * max(depth - slop, 0)
/// 2. Build effective mass matrix A = J * M^-1 * J^T
/// 3. Solve A * lambda_pos = b_pos with lambda_pos >= 0 (using ASM)
/// 4. Compute pseudo-velocity: dv_pseudo = M^-1 * J^T * lambda_pos
/// 5. Apply as position change: dx = dv_pseudo * dt
/// 6. Discard pseudo-velocity (never becomes kinetic energy)
///
/// @ticket 0040b_split_impulse_position_correction
class PositionCorrector
{
public:
  /// @brief Configuration parameters for position correction
  struct Config
  {
    double beta{0.2};      ///< Position correction factor [0, 1]
    double slop{0.005};    ///< Penetration tolerance [m] (no correction below)
    int maxIterations{4};  ///< Maximum position correction iterations
  };

  PositionCorrector() = default;
  ~PositionCorrector() = default;

  /// @brief Correct body positions to resolve penetration (default config)
  void correctPositions(const std::vector<Constraint*>& contactConstraints,
                        std::vector<InertialState*>& states,
                        const std::vector<double>& inverseMasses,
                        const std::vector<Eigen::Matrix3d>& inverseInertias,
                        size_t numBodies,
                        size_t numInertial,
                        double dt);

  /// @brief Correct body positions to resolve penetration
  ///
  /// Computes and applies position corrections for all active contacts.
  /// Only modifies position and orientation of bodies with non-zero
  /// inverse mass. Bodies with inverseMass == 0 (environment) are skipped.
  ///
  /// @param contactConstraints Active contact constraints with per-contact
  /// depth
  /// @param states Inertial states for all bodies (mutable for position
  /// updates)
  /// @param inverseMasses Per-body inverse mass [1/kg] (0 for static)
  /// @param inverseInertias Per-body inverse inertia tensors (world frame)
  /// @param numBodies Total bodies in solver (inertial + environment)
  /// @param numInertial Number of inertial (dynamic) bodies
  /// @param dt Timestep [s]
  /// @param config Position correction parameters
  void correctPositions(const std::vector<Constraint*>& contactConstraints,
                        std::vector<InertialState*>& states,
                        const std::vector<double>& inverseMasses,
                        const std::vector<Eigen::Matrix3d>& inverseInertias,
                        size_t numBodies,
                        size_t numInertial,
                        double dt,
                        const Config& config);

  // Rule of Five
  PositionCorrector(const PositionCorrector&) = default;
  PositionCorrector& operator=(const PositionCorrector&) = default;
  PositionCorrector(PositionCorrector&&) noexcept = default;
  PositionCorrector& operator=(PositionCorrector&&) noexcept = default;

private:
  static constexpr double kRegularizationEpsilon = 1e-8;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_POSITION_CORRECTOR_HPP
