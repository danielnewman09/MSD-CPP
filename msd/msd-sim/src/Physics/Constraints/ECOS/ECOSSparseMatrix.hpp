// Ticket: 0035b_box_constrained_asm_solver
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#ifndef MSD_SIM_PHYSICS_ECOS_SPARSE_MATRIX_HPP
#define MSD_SIM_PHYSICS_ECOS_SPARSE_MATRIX_HPP

#include <ecos/ecos.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <stdexcept>
#include <cmath>

namespace msd_sim
{

/**
 * @brief Convert Eigen matrices to ECOS CSC (Compressed Sparse Column) format
 *
 * ECOS requires sparse matrices in CSC format with specific types:
 * - data: pfloat* (typically double)
 * - row_indices: idxint* (typically long long when DLONG defined)
 * - col_ptrs: idxint* (size: ncol+1)
 *
 * CSC format stores non-zero values column-by-column:
 * - col_ptrs[j] = index in data/row_indices where column j starts
 * - col_ptrs[j+1] - col_ptrs[j] = number of non-zeros in column j
 * - row_indices[k] = row index of data[k]
 * - data[k] = matrix value at (row_indices[k], col)
 *
 * Example for matrix:
 *   [1.0  0.0  2.0]
 *   [0.0  3.0  0.0]
 *   [4.0  0.0  5.0]
 *
 * CSC representation:
 *   data = [1.0, 4.0, 3.0, 2.0, 5.0]
 *   row_indices = [0, 2, 1, 0, 2]
 *   col_ptrs = [0, 2, 3, 5]  (col 0 has 2 entries, col 1 has 1, col 2 has 2)
 *
 * Thread safety: Stateless conversion functions (thread-safe)
 * Error handling: Throws std::invalid_argument for invalid input
 * Memory: Data owned by ECOSSparseMatrix instance
 *
 * @see docs/designs/0035b_box_constrained_asm_solver/design.md
 * @ticket 0035b_box_constrained_asm_solver
 */
struct ECOSSparseMatrix
{
  std::vector<pfloat> data;           // Non-zero values
  std::vector<idxint> row_indices;    // Row index for each non-zero
  std::vector<idxint> col_ptrs;       // Column pointers (size: ncol+1)
  idxint nrows{0};                      // Number of rows
  idxint ncols{0};                      // Number of columns
  idxint nnz{0};                        // Number of non-zeros

  /**
   * @brief Default constructor creates empty matrix
   */
  ECOSSparseMatrix() = default;

  /**
   * @brief Convert Eigen dense matrix to CSC format
   *
   * Iterates column-major (Eigen default) and collects non-zero entries
   * with magnitude > sparsity_threshold.
   *
   * @param mat Dense matrix to convert
   * @param sparsity_threshold Values with |value| <= threshold are treated as zero (default: 1e-12)
   * @return ECOSSparseMatrix with CSC representation
   * @throws std::invalid_argument if matrix dimensions invalid
   */
  static ECOSSparseMatrix fromDense(const Eigen::MatrixXd& mat,
                                    double sparsity_threshold = 1e-12);

  /**
   * @brief Convert Eigen sparse matrix to CSC format
   *
   * Eigen sparse matrices are stored in column-major CSC by default,
   * enabling direct conversion.
   *
   * @param mat Sparse matrix to convert
   * @return ECOSSparseMatrix with CSC representation
   * @throws std::invalid_argument if matrix dimensions invalid
   */
  static ECOSSparseMatrix fromSparse(const Eigen::SparseMatrix<double>& mat);

  // Rule of Zero (compiler-generated copy/move/destructor)
  ECOSSparseMatrix(const ECOSSparseMatrix&) = default;
  ECOSSparseMatrix& operator=(const ECOSSparseMatrix&) = default;
  ECOSSparseMatrix(ECOSSparseMatrix&&) noexcept = default;
  ECOSSparseMatrix& operator=(ECOSSparseMatrix&&) noexcept = default;
  ~ECOSSparseMatrix() = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_ECOS_SPARSE_MATRIX_HPP
