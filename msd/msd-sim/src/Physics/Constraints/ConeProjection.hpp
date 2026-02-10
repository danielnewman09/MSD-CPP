// Ticket: 0052b_cone_projection_and_linear_algebra
// Design: docs/designs/0052_custom_friction_cone_solver/design.md

#ifndef MSD_SIM_PHYSICS_CONE_PROJECTION_HPP
#define MSD_SIM_PHYSICS_CONE_PROJECTION_HPP

#include <Eigen/Dense>

#include <vector>

namespace msd_sim
{

/**
 * @brief Result of projecting a single contact's impulse onto its friction cone
 *
 * @ticket 0052b_cone_projection_and_linear_algebra
 */
struct ProjectionResult
{
  double lambda_n{0.0};   // Normal impulse component
  double lambda_t1{0.0};  // First tangential impulse component
  double lambda_t2{0.0};  // Second tangential impulse component
  int case_{0};           // 1 = interior, 2 = origin, 3 = cone surface
};

/**
 * @brief Stateless utility providing Euclidean projection onto the friction cone
 *
 * Projects a 3-vector (lambda_n, lambda_t1, lambda_t2) onto the friction cone
 * K_mu = { (lambda_n, lambda_t1, lambda_t2) :
 *     ||(lambda_t1, lambda_t2)|| <= mu * lambda_n, lambda_n >= 0 }
 * and computes the projection's Jacobian matrix.
 *
 * Three geometric cases from math formulation M3:
 *   Case 1: ||p_t|| <= mu * p_n  -> identity (interior)
 *   Case 2: mu * ||p_t|| <= -p_n -> zero (dual cone)
 *   Case 3: otherwise            -> cone surface (Eq. 24-25)
 *
 * Thread safety: Stateless, all methods are static. Thread-safe.
 * Error handling: No exceptions. Handles degenerate inputs (zero mu, zero tangent).
 *
 * @see docs/designs/0052_custom_friction_cone_solver/design.md
 * @ticket 0052b_cone_projection_and_linear_algebra
 */
class ConeProjection
{
public:
  ConeProjection() = delete;  // Stateless utility, no instances

  /**
   * @brief Project (lambda_n, lambda_t1, lambda_t2) onto cone K_mu
   *
   * @param lambda_n Normal impulse component
   * @param lambda_t1 First tangential impulse component
   * @param lambda_t2 Second tangential impulse component
   * @param mu Friction coefficient (>= 0)
   * @return ProjectionResult with projected values and case identifier
   *
   * @ticket 0052b_cone_projection_and_linear_algebra
   */
  [[nodiscard]] static ProjectionResult project(
    double lambda_n, double lambda_t1, double lambda_t2, double mu);

  /**
   * @brief Compute 3x3 Jacobian of the cone projection
   *
   * Case 1: I_3 (identity), Case 2: 0 (zero), Case 3: rank-2 matrix (Eq. 32-33)
   *
   * @param lambda_n Normal impulse component
   * @param lambda_t1 First tangential impulse component
   * @param lambda_t2 Second tangential impulse component
   * @param mu Friction coefficient (>= 0)
   * @return 3x3 Jacobian matrix of the projection
   *
   * @ticket 0052b_cone_projection_and_linear_algebra
   */
  [[nodiscard]] static Eigen::Matrix3d gradient(
    double lambda_n, double lambda_t1, double lambda_t2, double mu);

  /**
   * @brief Project entire lambda vector (3C x 1) onto product cone K
   *
   * Processes C independent 3-tuples at indices [3i, 3i+1, 3i+2].
   *
   * @param lambda Stacked impulse vector (3C x 1)
   * @param mu Per-contact friction coefficients (C entries)
   * @param numContacts Number of contacts (C)
   * @return Projected lambda vector (3C x 1)
   *
   * @ticket 0052b_cone_projection_and_linear_algebra
   */
  [[nodiscard]] static Eigen::VectorXd projectVector(
    const Eigen::VectorXd& lambda,
    const std::vector<double>& mu,
    int numContacts);

private:
  static constexpr double kNearZeroTolerance = 1e-15;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONE_PROJECTION_HPP
