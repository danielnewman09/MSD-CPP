// Ticket: 0035b3_ecos_problem_construction
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINTS_ECOS_ECOS_PROBLEM_BUILDER_HPP
#define MSD_SIM_PHYSICS_CONSTRAINTS_ECOS_ECOS_PROBLEM_BUILDER_HPP

#include "msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp"
#include <Eigen/Dense>
#include <vector>

namespace msd_sim
{

/**
 * @brief Utility class for constructing ECOS problem data from contact constraints
 *
 * ECOSProblemBuilder converts a contact constraint system with friction into
 * ECOS standard form for second-order cone programming (SOCP). The contact problem:
 *
 *   A·λ = b, subject to ||λ_t_i|| ≤ μ_i·λ_n_i for all contacts i
 *
 * is reformulated as:
 *
 *   min c^T·x  s.t.  G·x + s = h,  s ∈ K
 *
 * where K is a product of second-order cones (one 3D cone per contact).
 *
 * **Key responsibilities**:
 * - Convert effective mass matrix A to ECOS sparse CSC format
 * - Build cone constraint matrix G that maps λ to cone slack variables
 * - Build cone RHS vector h (zeros for standard friction cone)
 * - Set up linear objective c (zeros for LCP formulation)
 * - Return populated ECOSData ready for ECOS_setup()
 *
 * **Mathematical formulation**:
 * Per contact i, the friction cone constraint is:
 *   ||[λ_t1^(i), λ_t2^(i)]|| ≤ μ_i·λ_n^(i)
 *
 * In ECOS notation (s_0 = μ·λ_n, s_1 = λ_t1, s_2 = λ_t2):
 *   ||[s_1, s_2]|| ≤ s_0  (3-dimensional second-order cone)
 *
 * Matrix G construction (block-diagonal, 3C × 3C):
 * For contact i at indices [3i, 3i+1, 3i+2]:
 *   Row 3i:   -μ_i at column 3i   (normal),    so s_0 = h_0 - (-μ_i·λ_n) = μ_i·λ_n
 *   Row 3i+1: -1   at column 3i+1 (tangent 1), so s_1 = h_1 - (-λ_t1)   = λ_t1
 *   Row 3i+2: -1   at column 3i+2 (tangent 2), so s_2 = h_2 - (-λ_t2)   = λ_t2
 *
 * h vector = 0 (standard friction cone with no offset)
 *
 * Equality constraints encode A·λ = b (LCP equality).
 *
 * @see docs/designs/0035b_box_constrained_asm_solver/design.md — "ECOS Problem Construction"
 * @ticket 0035b3_ecos_problem_construction
 */
class ECOSProblemBuilder
{
public:
  /**
   * @brief Build ECOS problem from contact constraint data
   *
   * Constructs ECOS problem data (G matrix, h vector, c vector, cone sizes)
   * from contact constraint system (A, b, friction coefficients). The returned
   * ECOSData is populated and ready for ECOSData::setup().
   *
   * **Preconditions**:
   * - A must be square (3C × 3C where C = number of contacts)
   * - b must have dimension 3C
   * - coneSpec must specify C cones (one per contact)
   * - A must be symmetric positive semi-definite (effective mass matrix)
   *
   * **Postconditions**:
   * - G matrix is block-diagonal (3C × 3C)
   * - h vector is zero (3C elements)
   * - c vector is zero (3C elements)
   * - cone_sizes is [3, 3, ..., 3] (C entries)
   *
   * @param A Effective mass matrix (3C × 3C), symmetric positive semi-definite
   * @param b RHS vector (3C × 1) with restitution and Baumgarte terms
   * @param coneSpec Friction cone specification (μ per contact, normal indices)
   * @return ECOSData populated and ready for setup()
   *
   * @throws std::invalid_argument if:
   *   - A is not square
   *   - A dimensions do not match b size
   *   - A dimensions do not match 3*coneSpec.numContacts
   *   - coneSpec.numContacts <= 0
   */
  static ECOSData build(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& b,
      const FrictionConeSpec& coneSpec);

private:
  /**
   * @brief Build block-diagonal G matrix for friction cone constraints
   *
   * Constructs the cone constraint matrix G (3C × 3C) that maps decision
   * variables λ to cone slack variables s. For standard friction cones,
   * G has a specific block-diagonal structure with -μ_i and -1 entries.
   *
   * **Block structure** (per contact i):
   * G_{3i, 3i}     = -μ_i  (normal scaling)
   * G_{3i+1, 3i+1} = -1    (tangent 1 passthrough)
   * G_{3i+2, 3i+2} = -1    (tangent 2 passthrough)
   * All other entries zero.
   *
   * **Rationale**: G·λ + s = h with h = 0 gives:
   *   s_0 = -(-μ_i·λ_n) = μ_i·λ_n
   *   s_1 = -(-λ_t1)    = λ_t1
   *   s_2 = -(-λ_t2)    = λ_t2
   * So the cone constraint ||[s_1, s_2]|| ≤ s_0 becomes ||[λ_t1, λ_t2]|| ≤ μ_i·λ_n.
   *
   * @param numContacts Number of contacts (C)
   * @param coneSpec Friction cone specification with μ values
   * @return ECOSSparseMatrix representing block-diagonal G matrix
   */
  static ECOSSparseMatrix buildGMatrix(
      idxint numContacts,
      const FrictionConeSpec& coneSpec);

};

// Note: Equality constraint matrix A_eq will be added in ticket 0035b4 when
// integrating the full ECOS problem formulation with the solver.

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINTS_ECOS_ECOS_PROBLEM_BUILDER_HPP
