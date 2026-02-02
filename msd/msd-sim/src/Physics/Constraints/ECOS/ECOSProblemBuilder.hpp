// Ticket: 0035b3_ecos_problem_construction
// Ticket: 0035d1_ecos_socp_reformulation
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md
// Design: docs/designs/0035d1_ecos_socp_reformulation/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINTS_ECOS_ECOS_PROBLEM_BUILDER_HPP
#define MSD_SIM_PHYSICS_CONSTRAINTS_ECOS_ECOS_PROBLEM_BUILDER_HPP

#include "msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp"
#include <Eigen/Dense>
#include <vector>

namespace msd_sim
{

/**
 * @brief Precomputed data for epigraph cone constraint
 *
 * Internal struct used by ECOSProblemBuilder to cache expensive
 * computations (Cholesky solve, matrix scaling) needed for epigraph
 * cone construction.
 *
 * @ticket 0035d1_ecos_socp_reformulation
 */
struct EpigraphConeData
{
  Eigen::MatrixXd L_times_2;   // 2·L (3C × 3C)
  Eigen::VectorXd d_times_2;   // 2·d where d = L⁻¹b (3C × 1), computed via forward substitution
  idxint epigraph_cone_size;   // Always 3C+2

  EpigraphConeData() = default;
};

/**
 * @brief Utility class for constructing ECOS problem data from contact constraints
 *
 * ECOSProblemBuilder converts a contact constraint system with friction into
 * ECOS standard form for second-order cone programming (SOCP). The contact problem
 * is reformulated as a Quadratic Program (QP) with conic constraints, lifted into
 * SOCP form using epigraph reformulation because ECOS is a conic solver, not a QP solver.
 *
 * **Correct formulation (Ticket 0035d1 — auxiliary-variable SOCP)**:
 *   Variables: x = [λ (3C); y (3C); t (1)] — total 6C+1
 *   min t
 *   s.t.  Lᵀλ - y = 0                               (3C equality constraints)
 *         ||(2(y - d), t-1)|| ≤ t+1                  (epigraph cone, d = L⁻¹b)
 *         ||[λ_t1_i, λ_t2_i]|| ≤ μ_i·λ_n_i          (friction cones)
 *
 * where A = L·Lᵀ (Cholesky), d = L⁻¹b (forward substitution), y are auxiliary
 * variables linking to λ, and t is the epigraph variable.
 *
 * **Key responsibilities**:
 * - Compute Cholesky factorization of effective mass matrix A
 * - Build equality constraints Lᵀλ - y = 0 (auxiliary-variable link)
 * - Build epigraph cone constraint for QP objective lifting (operates on y)
 * - Build extended G matrix with epigraph + friction cones
 * - Build extended h, c vectors
 * - Return populated ECOSData ready for ECOS_setup()
 *
 * **Mathematical formulation**:
 * Per contact i, the friction cone constraint is:
 *   ||[λ_t1^(i), λ_t2^(i)]|| ≤ μ_i·λ_n^(i)
 *
 * In ECOS notation (s_0 = μ·λ_n, s_1 = λ_t1, s_2 = λ_t2):
 *   ||[s_1, s_2]|| ≤ s_0  (3-dimensional second-order cone)
 *
 * **Extended variable vector**: x = [λ; y; t] (dimension 6C+1)
 * **Extended G matrix**: (6C+2) × (6C+1) — epigraph cone + friction cones
 * **Equality constraints**: A_eq (3C × (6C+1)) encodes Lᵀλ - y = 0
 * **Cone structure**: [3C+2, 3, 3, ..., 3] — one epigraph cone + C friction cones
 *
 * @see docs/designs/0035b_box_constrained_asm_solver/design.md — "ECOS Problem Construction"
 * @see docs/designs/0035d1_ecos_socp_reformulation/design.md — "ECOS SOCP Reformulation"
 * @see docs/designs/0035d1_ecos_socp_reformulation/math-formulation.md — "Mathematical Derivation"
 * @ticket 0035b3_ecos_problem_construction
 * @ticket 0035d1_ecos_socp_reformulation
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
   * **Ticket 0035d1**: Uses epigraph reformulation to convert QP with friction cones
   * to SOCP. The equality constraint A·λ = b is removed (it was over-constraining).
   * Instead, the QP objective (1/2)λᵀAλ - bᵀλ is lifted into SOCP form using an
   * epigraph variable t.
   *
   * **Preconditions**:
   * - A must be square (3C × 3C where C = number of contacts)
   * - b must have dimension 3C
   * - coneSpec must specify C cones (one per contact)
   * - A must be symmetric positive definite (effective mass matrix)
   *
   * **Postconditions**:
   * - G matrix is (6C+2) × (6C+1) with epigraph + friction cones
   * - h vector has dimension 6C+2 with epigraph RHS [1; -2d; -1; 0; ...] where d = L⁻¹b
   * - c vector has dimension 6C+1 with [0; ...; 0; 1] (minimize t)
   * - cone_sizes is [3C+2, 3, 3, ..., 3] (epigraph cone + C friction cones)
   * - A_eq is 3C × (6C+1) encoding Lᵀλ - y = 0
   * - b_eq is 3C zeros
   * - num_equality = 3C
   *
   * @param A Effective mass matrix (3C × 3C), symmetric positive definite
   * @param b RHS vector (3C × 1) with restitution and Baumgarte terms
   * @param coneSpec Friction cone specification (μ per contact, normal indices)
   * @return ECOSData populated and ready for setup()
   *
   * @throws std::invalid_argument if:
   *   - A is not square
   *   - A dimensions do not match b size
   *   - A dimensions do not match 3*coneSpec.numContacts
   *   - coneSpec.numContacts <= 0
   * @throws std::runtime_error if Cholesky factorization fails after regularization
   *
   * @ticket 0035d1_ecos_socp_reformulation
   */
  static ECOSData build(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& b,
      const FrictionConeSpec& coneSpec);

private:
  /**
   * @brief Compute Cholesky factorization of effective mass matrix
   *
   * Computes L such that A = L·Lᵀ (lower triangular). If factorization fails
   * due to ill-conditioning (condition number > 1e8), applies regularization:
   * A_reg = A + ε·I with ε = 1e-8 and retries.
   *
   * Reuses existing regularization logic from ConstraintSolver.cpp lines 441-452
   * (applyRegularizationFallback method pattern).
   *
   * @param A Effective mass matrix (3C × 3C), symmetric positive definite
   * @return L Cholesky factor (3C × 3C lower triangular)
   * @throws std::runtime_error if factorization fails after regularization
   *
   * @ticket 0035d1_ecos_socp_reformulation
   */
  static Eigen::MatrixXd computeCholeskyFactor(const Eigen::MatrixXd& A);

  /**
   * @brief Precompute epigraph cone constraint terms
   *
   * Computes precomputed terms for the epigraph SOC constraint:
   *   ||(2(y - d), t-1)|| ≤ t+1   where d = L⁻¹b
   *
   * Returns:
   * - L_times_2: Matrix 2·L for equality constraint construction
   * - d_times_2: Vector 2·d = 2·L⁻¹b for h vector constant (forward substitution)
   * - epigraph_cone_size: Always 3C+2 (dimension of epigraph cone)
   *
   * @param L Cholesky factor (3C × 3C)
   * @param b RHS vector (3C × 1)
   * @param numContacts Number of contacts (C)
   * @return EpigraphConeData with precomputed terms
   *
   * @ticket 0035d1_ecos_socp_reformulation
   */
  static EpigraphConeData buildEpigraphCone(
      const Eigen::MatrixXd& L,
      const Eigen::VectorXd& b,
      int numContacts);

  /**
   * @brief Build extended G matrix with epigraph and friction cones
   *
   * Constructs G matrix (6C+2 rows × (6C+1) cols) in CSC format:
   * - Rows 0 to 3C+1: Epigraph cone constraint (operates on y and t variables)
   * - Rows 3C+2 to 6C+1: Friction cone constraints (operates on λ variables)
   *
   * Block structure:
   *   G = [ G_epi  ]    (3C+2) × (6C+1)  epigraph on [y; t]
   *       [ G_fric ]    (3C)   × (6C+1)  friction on λ, zero on [y; t]
   *
   * @param epigraphData Precomputed epigraph cone terms
   * @param coneSpec Friction cone specification (μ values)
   * @param numContacts Number of contacts (C)
   * @return ECOSSparseMatrix in CSC format
   *
   * @ticket 0035d1_ecos_socp_reformulation
   */
  static ECOSSparseMatrix buildExtendedGMatrix(
      const EpigraphConeData& epigraphData,
      const FrictionConeSpec& coneSpec,
      int numContacts);

  /**
   * @brief Build extended h vector (cone RHS)
   *
   * Constructs h vector (6C+2 elements):
   * - Rows 0 to 3C+1: Epigraph RHS [1; -2d; -1] where d = L⁻¹b
   * - Rows 3C+2 to 6C+1: Friction RHS [0, 0, 0, ...] (unchanged)
   *
   * @param epigraphData Precomputed epigraph cone terms
   * @param numContacts Number of contacts (C)
   * @return std::vector<pfloat> of size 6C+2
   *
   * @ticket 0035d1_ecos_socp_reformulation
   */
  static std::vector<pfloat> buildExtendedHVector(
      const EpigraphConeData& epigraphData,
      int numContacts);

  /**
   * @brief Build extended c vector (objective)
   *
   * Constructs c vector (6C+1 elements):
   * - Elements 0 to 3C-1: Zero (no cost on λ)
   * - Elements 3C to 6C-1: Zero (no cost on y)
   * - Element 6C: 1.0 (minimize epigraph variable t)
   *
   * @param numContacts Number of contacts (C)
   * @return std::vector<pfloat> of size 6C+1
   *
   * @ticket 0035d1_ecos_socp_reformulation
   */
  static std::vector<pfloat> buildExtendedCVector(int numContacts);

};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINTS_ECOS_ECOS_PROBLEM_BUILDER_HPP
