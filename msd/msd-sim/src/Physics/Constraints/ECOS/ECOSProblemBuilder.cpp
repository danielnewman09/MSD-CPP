// Ticket: 0035b3_ecos_problem_construction
// Ticket: 0035d1_ecos_socp_reformulation
// Design: docs/designs/0035d1_ecos_socp_reformulation/design.md

#include "msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp"
#include <Eigen/Cholesky>
#include <stdexcept>
#include <sstream>
#include <limits>
#include <cmath>

namespace msd_sim
{

// ===== Cholesky Factorization (Ticket 0035d1) =====

Eigen::MatrixXd ECOSProblemBuilder::computeCholeskyFactor(
    const Eigen::MatrixXd& A)
{
  // Fixed regularization epsilon for Cholesky. Applied only if factorization
  // fails (matrix is not positive definite).
  constexpr double kRegularizationEpsilon = 1.0e-8;

  Eigen::LLT<Eigen::MatrixXd> llt{A};

  if (llt.info() == Eigen::Success)
  {
    return llt.matrixL();
  }

  // Factorization failed — apply regularization: A_reg = A + ε·I
  Eigen::MatrixXd A_reg = A;
  for (Eigen::Index i = 0; i < A_reg.rows(); ++i)
  {
    A_reg(i, i) += kRegularizationEpsilon;
  }

  Eigen::LLT<Eigen::MatrixXd> llt_reg{A_reg};

  if (llt_reg.info() != Eigen::Success)
  {
    throw std::runtime_error{
        "ECOSProblemBuilder::computeCholeskyFactor: "
        "Cholesky factorization failed after regularization (epsilon = 1e-8)"};
  }

  return llt_reg.matrixL();
}

// ===== Epigraph Cone Precomputation (Ticket 0035d1) =====
//
// Auxiliary-variable formulation:
//   Variables: x = [λ (3C); y (3C); t (1)] — total 6C+1
//   Equality: Lᵀλ - y = 0 (links auxiliary y to physical λ)
//   Epigraph cone: ||(2(y - d), t-1)|| ≤ t+1 where d = L⁻¹b (stable fwd sub)
//   Friction cones: ||[λ_t1, λ_t2]|| ≤ μ λ_n (on λ variables)
//
// This avoids computing L⁻ᵀb (numerically unstable backward substitution
// through a potentially ill-conditioned upper triangular matrix). Instead,
// d = L⁻¹b is computed by stable forward substitution.

EpigraphConeData ECOSProblemBuilder::buildEpigraphCone(
    const Eigen::MatrixXd& L,
    const Eigen::VectorXd& b,
    int numContacts)
{
  EpigraphConeData data{};

  // 2·L is no longer needed for the G matrix epigraph block (the epigraph
  // now operates on y variables directly). Store it for the equality constraint.
  data.L_times_2 = 2.0 * L;

  // Compute d = L⁻¹b via forward substitution (numerically stable).
  // The h vector uses 2d = 2·L⁻¹b.
  Eigen::VectorXd d = L.triangularView<Eigen::Lower>().solve(b);
  data.d_times_2 = 2.0 * d;

  // Epigraph cone dimension: 3C + 2
  // [t+1; 2(y-d); t-1] ∈ Q^{3C+2}
  data.epigraph_cone_size = static_cast<idxint>(3 * numContacts + 2);

  return data;
}

// ===== Extended G Matrix (Ticket 0035d1) =====
//
// Variable layout: x = [λ (3C); y (3C); t (1)]
// G matrix structure (total rows = (3C+2) + 3C):
//
// Epigraph cone (3C+2 rows):
//   Row 0:         [0..0 (λ), 0..0 (y), -1 (t)]          s₀ = 1 - (-1)t = 1+t
//   Rows 1..3C:    [0..0 (λ), -2I (y),   0 (t)]          s_i = -2d_i - (-2)y_i = 2(y_i - d_i)
//   Row 3C+1:      [0..0 (λ), 0..0 (y), -1 (t)]          s_{3C+1} = -1 - (-1)t = t-1
//
// Friction cones (3C rows, on λ variables):
//   Per contact i:
//     Row 3C+2+3i:   [-μ_i at col 3i, rest 0]
//     Row 3C+2+3i+1: [-1 at col 3i+1, rest 0]
//     Row 3C+2+3i+2: [-1 at col 3i+2, rest 0]

ECOSSparseMatrix ECOSProblemBuilder::buildExtendedGMatrix(
    const EpigraphConeData& epigraphData,
    const FrictionConeSpec& coneSpec,
    int numContacts)
{
  const int C = numContacts;
  const int lambdaDim = 3 * C;
  const int yDim = 3 * C;
  const int numVars = lambdaDim + yDim + 1;  // [λ; y; t]
  const int epiRows = lambdaDim + 2;         // 3C+2
  const int fricRows = 3 * C;                // 3C
  const int totalRows = epiRows + fricRows;  // 6C+2

  // Count non-zeros:
  // Epigraph block:
  //   Row 0: 1 nnz (t column, value -1)
  //   Rows 1..3C: 3C nnz (diagonal -2 in y columns)
  //   Row 3C+1: 1 nnz (t column, value -1)
  // Friction block: 3C nnz (diagonal -μ or -1 in λ columns)
  // Total: 2 + 3C + 3C = 6C + 2
  const idxint totalNnz = static_cast<idxint>(6 * C + 2);

  ECOSSparseMatrix G{};
  G.nrows = static_cast<idxint>(totalRows);
  G.ncols = static_cast<idxint>(numVars);
  G.nnz = totalNnz;

  G.data.reserve(static_cast<size_t>(totalNnz));
  G.row_indices.reserve(static_cast<size_t>(totalNnz));
  G.col_ptrs.reserve(static_cast<size_t>(numVars + 1));

  // Build CSC column-by-column

  // --- Lambda columns (0..3C-1): only friction entries ---
  for (int col = 0; col < lambdaDim; ++col)
  {
    G.col_ptrs.push_back(static_cast<idxint>(G.data.size()));

    const int contactIdx = col / 3;
    const int componentIdx = col % 3;
    const int fricRow = epiRows + 3 * contactIdx + componentIdx;

    if (componentIdx == 0)
    {
      const double mu = coneSpec.getFrictionCoefficient(contactIdx);
      G.data.push_back(static_cast<pfloat>(-mu));
    }
    else
    {
      G.data.push_back(static_cast<pfloat>(-1.0));
    }
    G.row_indices.push_back(static_cast<idxint>(fricRow));
  }

  // --- y columns (3C..6C-1): epigraph diagonal entries ---
  for (int yIdx = 0; yIdx < yDim; ++yIdx)
  {
    G.col_ptrs.push_back(static_cast<idxint>(G.data.size()));

    // Epigraph row (yIdx + 1): -2
    G.row_indices.push_back(static_cast<idxint>(yIdx + 1));
    G.data.push_back(static_cast<pfloat>(-2.0));
  }

  // --- t column (last): two epigraph entries ---
  G.col_ptrs.push_back(static_cast<idxint>(G.data.size()));

  // Row 0: -1
  G.row_indices.push_back(0);
  G.data.push_back(static_cast<pfloat>(-1.0));

  // Row 3C+1: -1
  G.row_indices.push_back(static_cast<idxint>(lambdaDim + 1));
  G.data.push_back(static_cast<pfloat>(-1.0));

  // Final column pointer
  G.col_ptrs.push_back(static_cast<idxint>(G.data.size()));
  G.nnz = static_cast<idxint>(G.data.size());

  return G;
}

// ===== Extended h Vector (Ticket 0035d1) =====
//
// h vector (6C+2 elements):
//   h[0] = 1                    (epigraph: s₀ = t+1)
//   h[1..3C] = -2d = -2L⁻¹b   (epigraph: s_i = 2(y_i - d_i))
//   h[3C+1] = -1               (epigraph: s_{3C+1} = t-1)
//   h[3C+2..6C+1] = 0          (friction cones)

std::vector<pfloat> ECOSProblemBuilder::buildExtendedHVector(
    const EpigraphConeData& epigraphData,
    int numContacts)
{
  const int lambdaDim = 3 * numContacts;
  const int epiRows = lambdaDim + 2;
  const int fricRows = 3 * numContacts;
  const int totalRows = epiRows + fricRows;

  std::vector<pfloat> h(static_cast<size_t>(totalRows), 0.0);

  h[0] = 1.0;

  for (int i = 0; i < lambdaDim; ++i)
  {
    // h[i+1] = -2d_i where d = L⁻¹b (stored in d_times_2 as 2d)
    h[static_cast<size_t>(i + 1)] =
        static_cast<pfloat>(-epigraphData.d_times_2(i));
  }

  h[static_cast<size_t>(lambdaDim + 1)] = -1.0;

  return h;
}

// ===== Extended c Vector (Ticket 0035d1) =====
//
// c vector (6C+1 elements):
//   c[0..3C-1] = 0    (no cost on λ)
//   c[3C..6C-1] = 0   (no cost on y)
//   c[6C] = 1         (minimize t)

std::vector<pfloat> ECOSProblemBuilder::buildExtendedCVector(int numContacts)
{
  const int numVars = 6 * numContacts + 1;  // [λ; y; t]
  std::vector<pfloat> c(static_cast<size_t>(numVars), 0.0);
  c[static_cast<size_t>(numVars - 1)] = 1.0;
  return c;
}

// ===== Main Build Method (Ticket 0035d1) =====

ECOSData ECOSProblemBuilder::build(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const FrictionConeSpec& coneSpec)
{
  // Validate inputs
  if (A.rows() != A.cols())
  {
    std::ostringstream oss;
    oss << "ECOSProblemBuilder::build: A matrix must be square, got "
        << A.rows() << " x " << A.cols();
    throw std::invalid_argument{oss.str()};
  }

  const int numContacts = coneSpec.getNumContacts();
  if (numContacts <= 0)
  {
    throw std::invalid_argument{
        "ECOSProblemBuilder::build: numContacts must be positive"};
  }

  const int lambdaDim = 3 * numContacts;
  if (A.rows() != lambdaDim)
  {
    std::ostringstream oss;
    oss << "ECOSProblemBuilder::build: A matrix dimension " << A.rows()
        << " does not match 3*numContacts = " << lambdaDim;
    throw std::invalid_argument{oss.str()};
  }

  if (b.size() != lambdaDim)
  {
    std::ostringstream oss;
    oss << "ECOSProblemBuilder::build: b vector size " << b.size()
        << " does not match 3*numContacts = " << lambdaDim;
    throw std::invalid_argument{oss.str()};
  }

  // Step 1: Cholesky factorization A = L·Lᵀ (with regularization fallback)
  Eigen::MatrixXd L = computeCholeskyFactor(A);

  // Step 2: Precompute epigraph cone terms (uses L⁻¹b via forward sub)
  EpigraphConeData epigraphData = buildEpigraphCone(L, b, numContacts);

  // Step 3: Build extended problem matrices
  ECOSSparseMatrix G = buildExtendedGMatrix(epigraphData, coneSpec, numContacts);
  std::vector<pfloat> h = buildExtendedHVector(epigraphData, numContacts);
  std::vector<pfloat> c = buildExtendedCVector(numContacts);

  // Step 4: Build cone sizes: [3C+2, 3, 3, ..., 3]
  std::vector<idxint> coneSizes;
  coneSizes.reserve(static_cast<size_t>(numContacts + 1));
  coneSizes.push_back(epigraphData.epigraph_cone_size);
  auto frictionCones = coneSpec.getConeSizes();
  coneSizes.insert(coneSizes.end(), frictionCones.begin(), frictionCones.end());

  // Step 5: Build equality constraint matrices: Lᵀλ - y = 0
  // A_eq is 3C × (6C+1), b_eq is 3C × 1 (all zeros)
  //
  // A_eq = [Lᵀ | -I | 0]
  // This links the auxiliary variable y to the physical λ via y = Lᵀλ
  const int numEq = lambdaDim;  // 3C equality constraints
  const int numVars = 6 * numContacts + 1;

  // Build A_eq in CSC format
  ECOSSparseMatrix A_eq{};
  A_eq.nrows = static_cast<idxint>(numEq);
  A_eq.ncols = static_cast<idxint>(numVars);

  // Count nnz: Lᵀ is upper triangular 3C×3C → nnz = 3C*(3C+1)/2
  // Plus -I block: 3C diagonal entries
  // Plus t column: 0 entries
  // Total approximate nnz (count actual from L)
  idxint lTransposeNnz = 0;
  for (int col = 0; col < lambdaDim; ++col)
  {
    for (int row = 0; row <= col; ++row)  // Upper triangle of Lᵀ
    {
      if (std::abs(L(col, row)) > 1e-15)  // L(col,row) = Lᵀ(row,col)
      {
        ++lTransposeNnz;
      }
    }
  }
  const idxint aeqNnz = lTransposeNnz + static_cast<idxint>(lambdaDim);

  A_eq.data.reserve(static_cast<size_t>(aeqNnz));
  A_eq.row_indices.reserve(static_cast<size_t>(aeqNnz));
  A_eq.col_ptrs.reserve(static_cast<size_t>(numVars + 1));

  // Lambda columns (0..3C-1): entries from Lᵀ
  // Lᵀ(row, col) = L(col, row). For column j of A_eq, the entries are
  // Lᵀ(row, j) = L(j, row) for row = 0..j (upper triangle of Lᵀ means row ≤ col)
  for (int col = 0; col < lambdaDim; ++col)
  {
    A_eq.col_ptrs.push_back(static_cast<idxint>(A_eq.data.size()));

    for (int row = 0; row <= col; ++row)
    {
      double val = L(col, row);  // Lᵀ(row, col) = L(col, row)
      if (std::abs(val) > 1e-15)
      {
        A_eq.row_indices.push_back(static_cast<idxint>(row));
        A_eq.data.push_back(static_cast<pfloat>(val));
      }
    }
  }

  // y columns (3C..6C-1): -I diagonal
  for (int yIdx = 0; yIdx < lambdaDim; ++yIdx)
  {
    A_eq.col_ptrs.push_back(static_cast<idxint>(A_eq.data.size()));
    A_eq.row_indices.push_back(static_cast<idxint>(yIdx));
    A_eq.data.push_back(static_cast<pfloat>(-1.0));
  }

  // t column: no equality entries
  A_eq.col_ptrs.push_back(static_cast<idxint>(A_eq.data.size()));

  // Final column pointer
  A_eq.col_ptrs.push_back(static_cast<idxint>(A_eq.data.size()));
  A_eq.nnz = static_cast<idxint>(A_eq.data.size());

  // b_eq = 0 (3C zeros)
  std::vector<pfloat> b_eq(static_cast<size_t>(numEq), 0.0);

  // Step 6: Create ECOSData
  // Variables: 6C+1 (λ + y + t)
  // Inequality rows: (3C+2) + 3C = 6C+2
  // Cones: C+1 (1 epigraph + C friction)
  // Equality: 3C (Lᵀλ - y = 0)
  const idxint numInequalityRows = static_cast<idxint>(G.nrows);
  const idxint numCones = static_cast<idxint>(numContacts + 1);

  ECOSData data{static_cast<idxint>(numVars), numInequalityRows, numCones,
                static_cast<idxint>(numEq)};

  data.G_ = std::move(G);
  data.h_ = std::move(h);
  data.c_ = std::move(c);
  data.cone_sizes_ = std::move(coneSizes);
  data.A_eq_ = std::move(A_eq);
  data.b_eq_ = std::move(b_eq);

  return data;
}

}  // namespace msd_sim
