// Ticket: 0035b_box_constrained_asm_solver
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#include "msd-sim/src/Physics/Constraints/ECOS/ECOSSparseMatrix.hpp"

namespace msd_sim
{

ECOSSparseMatrix ECOSSparseMatrix::fromDense(const Eigen::MatrixXd& mat,
                                             double sparsity_threshold)
{
  if (mat.rows() == 0 || mat.cols() == 0)
  {
    throw std::invalid_argument(
        "ECOSSparseMatrix::fromDense: matrix must have non-zero dimensions");
  }

  ECOSSparseMatrix result{};
  result.nrows = static_cast<idxint>(mat.rows());
  result.ncols = static_cast<idxint>(mat.cols());

  // Reserve space for column pointers (ncols + 1)
  result.col_ptrs.reserve(static_cast<size_t>(result.ncols) + 1);
  result.col_ptrs.push_back(0);  // First column starts at index 0

  // Iterate column-major (Eigen default storage order)
  for (idxint col = 0; col < result.ncols; ++col)
  {
    for (idxint row = 0; row < result.nrows; ++row)
    {
      double value = mat(row, col);
      if (std::abs(value) > sparsity_threshold)
      {
        result.data.push_back(static_cast<pfloat>(value));
        result.row_indices.push_back(row);
      }
    }

    // Column pointer for next column = current number of non-zeros
    result.col_ptrs.push_back(static_cast<idxint>(result.data.size()));
  }

  result.nnz = static_cast<idxint>(result.data.size());

  return result;
}

ECOSSparseMatrix ECOSSparseMatrix::fromSparse(const Eigen::SparseMatrix<double>& mat)
{
  if (mat.rows() == 0 || mat.cols() == 0)
  {
    throw std::invalid_argument(
        "ECOSSparseMatrix::fromSparse: matrix must have non-zero dimensions");
  }

  ECOSSparseMatrix result{};
  result.nrows = static_cast<idxint>(mat.rows());
  result.ncols = static_cast<idxint>(mat.cols());
  result.nnz = static_cast<idxint>(mat.nonZeros());

  // Reserve space
  result.data.reserve(static_cast<size_t>(result.nnz));
  result.row_indices.reserve(static_cast<size_t>(result.nnz));
  result.col_ptrs.reserve(static_cast<size_t>(result.ncols) + 1);

  // Eigen sparse matrices are column-major CSC by default
  // We can directly iterate over the compressed structure
  for (int col = 0; col < mat.outerSize(); ++col)
  {
    result.col_ptrs.push_back(static_cast<idxint>(result.data.size()));

    for (Eigen::SparseMatrix<double>::InnerIterator it{mat, col}; it; ++it)
    {
      result.data.push_back(static_cast<pfloat>(it.value()));
      result.row_indices.push_back(static_cast<idxint>(it.row()));
    }
  }

  // Final column pointer (total number of non-zeros)
  result.col_ptrs.push_back(static_cast<idxint>(result.data.size()));

  return result;
}

}  // namespace msd_sim
