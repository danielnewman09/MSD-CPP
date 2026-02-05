// Ticket: 0035b3_ecos_problem_construction
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#include "msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <ecos/glblopts.h>
#include <cstddef>
#include <stdexcept>
#include <sstream>
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp"
#include "msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp"

namespace msd_sim
{

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

  const idxint numContacts = coneSpec.getNumContacts();
  if (numContacts <= 0)
  {
    throw std::invalid_argument{
        "ECOSProblemBuilder::build: numContacts must be positive"};
  }

  const idxint expectedDim = 3 * numContacts;
  if (A.rows() != expectedDim)
  {
    std::ostringstream oss;
    oss << "ECOSProblemBuilder::build: A matrix dimension " << A.rows()
        << " does not match 3*numContacts = " << expectedDim;
    throw std::invalid_argument{oss.str()};
  }

  if (b.size() != expectedDim)
  {
    std::ostringstream oss;
    oss << "ECOSProblemBuilder::build: b vector size " << b.size()
        << " does not match 3*numContacts = " << expectedDim;
    throw std::invalid_argument{oss.str()};
  }

  // Create ECOSData with problem dimensions
  // Ticket 0035b4: include equality constraints (A·λ = b)
  ECOSData data{expectedDim, numContacts, expectedDim};

  // Build cone constraint matrix G (block-diagonal)
  data.G_ = buildGMatrix(numContacts, coneSpec);

  // Build cone RHS vector h (all zeros for standard friction cone)
  data.h_.assign(static_cast<size_t>(expectedDim), 0.0);

  // Build linear objective c (all zeros for feasibility problem)
  data.c_.assign(static_cast<size_t>(expectedDim), 0.0);

  // Build cone size array (all cones are dimension 3)
  data.cone_sizes_ = coneSpec.getConeSizes();

  // Build equality constraint matrix A_eq from effective mass matrix (Ticket 0035b4)
  // A_eq·λ = b_eq encodes the LCP equality A·λ = b
  data.A_eq_ = ECOSSparseMatrix::fromDense(A);

  // Build equality constraint RHS from b vector
  data.b_eq_.resize(static_cast<size_t>(expectedDim));
  for (idxint i = 0; i < expectedDim; ++i)
  {
    data.b_eq_[static_cast<size_t>(i)] = static_cast<pfloat>(b(i));
  }

  return data;
}

ECOSSparseMatrix ECOSProblemBuilder::buildGMatrix(
    idxint numContacts,
    const FrictionConeSpec& coneSpec)
{
  // G matrix is 3C × 3C block-diagonal
  // Each contact i contributes a 3×3 diagonal block at indices [3i:3i+2, 3i:3i+2]
  // Block structure:
  //   G_{3i, 3i}     = -μ_i  (normal scaling)
  //   G_{3i+1, 3i+1} = -1    (tangent 1)
  //   G_{3i+2, 3i+2} = -1    (tangent 2)
  // Total non-zeros: 3 per contact = 3*numContacts

  const idxint nrows = 3 * numContacts;
  const idxint ncols = 3 * numContacts;
  const idxint nnz = 3 * numContacts;

  ECOSSparseMatrix g{};
  g.nrows = nrows;
  g.ncols = ncols;
  g.nnz = nnz;

  g.data.reserve(static_cast<size_t>(nnz));
  g.row_indices.reserve(static_cast<size_t>(nnz));
  g.col_ptrs.reserve(static_cast<size_t>(ncols + 1));

  // Build CSC format column-by-column
  idxint currentNnz = 0;

  for (idxint col = 0; col < ncols; ++col)
  {
    g.col_ptrs.push_back(currentNnz);

    // Determine which contact this column belongs to
    const idxint contactIdx = col / 3;
    const idxint componentIdx = col % 3;  // 0 = normal, 1 = tangent1, 2 = tangent2

    // Each column has exactly one non-zero on the diagonal
    const idxint row = col;

    if (componentIdx == 0)
    {
      // Normal component: -μ_i
      const double mu = coneSpec.getFrictionCoefficient(static_cast<int>(contactIdx));
      g.data.push_back(static_cast<pfloat>(-mu));
    }
    else
    {
      // Tangent components: -1
      g.data.push_back(static_cast<pfloat>(-1.0));
    }

    g.row_indices.push_back(row);
    ++currentNnz;
  }

  // Final column pointer (points to end of data)
  g.col_ptrs.push_back(currentNnz);

  return g;
}

}  // namespace msd_sim
