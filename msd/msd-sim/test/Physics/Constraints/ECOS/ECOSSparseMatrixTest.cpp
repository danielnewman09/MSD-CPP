// Ticket: 0035b_box_constrained_asm_solver
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSSparseMatrix.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>

using namespace msd_sim;

namespace {
  // Helper to compare CSC representations
  bool csrEqual(const ECOSSparseMatrix& a, const ECOSSparseMatrix& b, double tol = 1e-12) {
    if (a.nrows != b.nrows || a.ncols != b.ncols || a.nnz != b.nnz) {
      return false;
    }

    if (a.data.size() != b.data.size() ||
        a.row_indices.size() != b.row_indices.size() ||
        a.col_ptrs.size() != b.col_ptrs.size()) {
      return false;
    }

    for (size_t i = 0; i < a.data.size(); ++i) {
      if (std::abs(a.data[i] - b.data[i]) > tol) {
        return false;
      }
    }

    for (size_t i = 0; i < a.row_indices.size(); ++i) {
      if (a.row_indices[i] != b.row_indices[i]) {
        return false;
      }
    }

    for (size_t i = 0; i < a.col_ptrs.size(); ++i) {
      if (a.col_ptrs[i] != b.col_ptrs[i]) {
        return false;
      }
    }

    return true;
  }
}

TEST(ECOSSparseMatrix, DefaultConstructorCreatesEmptyMatrix)
{
  ECOSSparseMatrix mat{};

  EXPECT_EQ(mat.nrows, 0);
  EXPECT_EQ(mat.ncols, 0);
  EXPECT_EQ(mat.nnz, 0);
  EXPECT_TRUE(mat.data.empty());
  EXPECT_TRUE(mat.row_indices.empty());
  EXPECT_TRUE(mat.col_ptrs.empty());
}

TEST(ECOSSparseMatrix, FromDenseThrowsForEmptyMatrix)
{
  Eigen::MatrixXd empty{0, 0};
  EXPECT_THROW(ECOSSparseMatrix::fromDense(empty), std::invalid_argument);
}

TEST(ECOSSparseMatrix, FromDenseHandlesSingleElementMatrix)
{
  Eigen::MatrixXd mat{1, 1};
  mat(0, 0) = 5.0;

  auto result = ECOSSparseMatrix::fromDense(mat);

  EXPECT_EQ(result.nrows, 1);
  EXPECT_EQ(result.ncols, 1);
  EXPECT_EQ(result.nnz, 1);
  ASSERT_EQ(result.data.size(), 1);
  EXPECT_DOUBLE_EQ(result.data[0], 5.0);
  ASSERT_EQ(result.row_indices.size(), 1);
  EXPECT_EQ(result.row_indices[0], 0);
  ASSERT_EQ(result.col_ptrs.size(), 2);
  EXPECT_EQ(result.col_ptrs[0], 0);
  EXPECT_EQ(result.col_ptrs[1], 1);
}

TEST(ECOSSparseMatrix, FromDenseHandlesIdentityMatrix)
{
  Eigen::MatrixXd mat = Eigen::MatrixXd::Identity(3, 3);

  auto result = ECOSSparseMatrix::fromDense(mat);

  EXPECT_EQ(result.nrows, 3);
  EXPECT_EQ(result.ncols, 3);
  EXPECT_EQ(result.nnz, 3);

  // Expected CSC for identity:
  // data = [1.0, 1.0, 1.0]
  // row_indices = [0, 1, 2]
  // col_ptrs = [0, 1, 2, 3]
  ASSERT_EQ(result.data.size(), 3);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(result.data[i], 1.0);
    EXPECT_EQ(result.row_indices[i], static_cast<idxint>(i));
  }

  ASSERT_EQ(result.col_ptrs.size(), 4);
  EXPECT_EQ(result.col_ptrs[0], 0);
  EXPECT_EQ(result.col_ptrs[1], 1);
  EXPECT_EQ(result.col_ptrs[2], 2);
  EXPECT_EQ(result.col_ptrs[3], 3);
}

TEST(ECOSSparseMatrix, FromDenseHandlesKnownSparsePattern)
{
  // Test matrix:
  //   [1.0  0.0  2.0]
  //   [0.0  3.0  0.0]
  //   [4.0  0.0  5.0]
  Eigen::MatrixXd mat{3, 3};
  mat << 1.0, 0.0, 2.0,
         0.0, 3.0, 0.0,
         4.0, 0.0, 5.0;

  auto result = ECOSSparseMatrix::fromDense(mat);

  EXPECT_EQ(result.nrows, 3);
  EXPECT_EQ(result.ncols, 3);
  EXPECT_EQ(result.nnz, 5);

  // Expected CSC:
  // Column 0: [1.0, 4.0] at rows [0, 2]
  // Column 1: [3.0] at row [1]
  // Column 2: [2.0, 5.0] at rows [0, 2]
  ASSERT_EQ(result.data.size(), 5);
  EXPECT_DOUBLE_EQ(result.data[0], 1.0);
  EXPECT_DOUBLE_EQ(result.data[1], 4.0);
  EXPECT_DOUBLE_EQ(result.data[2], 3.0);
  EXPECT_DOUBLE_EQ(result.data[3], 2.0);
  EXPECT_DOUBLE_EQ(result.data[4], 5.0);

  ASSERT_EQ(result.row_indices.size(), 5);
  EXPECT_EQ(result.row_indices[0], 0);
  EXPECT_EQ(result.row_indices[1], 2);
  EXPECT_EQ(result.row_indices[2], 1);
  EXPECT_EQ(result.row_indices[3], 0);
  EXPECT_EQ(result.row_indices[4], 2);

  ASSERT_EQ(result.col_ptrs.size(), 4);
  EXPECT_EQ(result.col_ptrs[0], 0);
  EXPECT_EQ(result.col_ptrs[1], 2);
  EXPECT_EQ(result.col_ptrs[2], 3);
  EXPECT_EQ(result.col_ptrs[3], 5);
}

TEST(ECOSSparseMatrix, FromDenseAppliesSparsityThreshold)
{
  Eigen::MatrixXd mat{2, 2};
  mat << 1.0, 1e-15,
         1e-13, 2.0;

  auto result = ECOSSparseMatrix::fromDense(mat, 1e-12);

  // Values below 1e-12 should be omitted
  EXPECT_EQ(result.nnz, 2);
  ASSERT_EQ(result.data.size(), 2);
  EXPECT_DOUBLE_EQ(result.data[0], 1.0);
  EXPECT_DOUBLE_EQ(result.data[1], 2.0);
}

TEST(ECOSSparseMatrix, FromDenseHandlesRectangularMatrix)
{
  Eigen::MatrixXd mat{2, 3};
  mat << 1.0, 0.0, 2.0,
         3.0, 4.0, 0.0;

  auto result = ECOSSparseMatrix::fromDense(mat);

  EXPECT_EQ(result.nrows, 2);
  EXPECT_EQ(result.ncols, 3);
  EXPECT_EQ(result.nnz, 4);

  // Column 0: [1.0, 3.0] at rows [0, 1]
  // Column 1: [4.0] at row [1]
  // Column 2: [2.0] at row [0]
  ASSERT_EQ(result.data.size(), 4);
  EXPECT_DOUBLE_EQ(result.data[0], 1.0);
  EXPECT_DOUBLE_EQ(result.data[1], 3.0);
  EXPECT_DOUBLE_EQ(result.data[2], 4.0);
  EXPECT_DOUBLE_EQ(result.data[3], 2.0);

  ASSERT_EQ(result.col_ptrs.size(), 4);
  EXPECT_EQ(result.col_ptrs[0], 0);
  EXPECT_EQ(result.col_ptrs[1], 2);
  EXPECT_EQ(result.col_ptrs[2], 3);
  EXPECT_EQ(result.col_ptrs[3], 4);
}

TEST(ECOSSparseMatrix, FromSparseThrowsForEmptyMatrix)
{
  Eigen::SparseMatrix<double> empty{0, 0};
  EXPECT_THROW(ECOSSparseMatrix::fromSparse(empty), std::invalid_argument);
}

TEST(ECOSSparseMatrix, FromSparseHandlesIdentityMatrix)
{
  Eigen::SparseMatrix<double> mat{3, 3};
  mat.setIdentity();

  auto result = ECOSSparseMatrix::fromSparse(mat);

  EXPECT_EQ(result.nrows, 3);
  EXPECT_EQ(result.ncols, 3);
  EXPECT_EQ(result.nnz, 3);

  ASSERT_EQ(result.data.size(), 3);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(result.data[i], 1.0);
    EXPECT_EQ(result.row_indices[i], static_cast<idxint>(i));
  }

  ASSERT_EQ(result.col_ptrs.size(), 4);
  EXPECT_EQ(result.col_ptrs[0], 0);
  EXPECT_EQ(result.col_ptrs[1], 1);
  EXPECT_EQ(result.col_ptrs[2], 2);
  EXPECT_EQ(result.col_ptrs[3], 3);
}

TEST(ECOSSparseMatrix, FromSparseHandlesKnownPattern)
{
  // Same test pattern as fromDense
  Eigen::SparseMatrix<double> mat{3, 3};
  mat.insert(0, 0) = 1.0;
  mat.insert(2, 0) = 4.0;
  mat.insert(1, 1) = 3.0;
  mat.insert(0, 2) = 2.0;
  mat.insert(2, 2) = 5.0;
  mat.makeCompressed();

  auto result = ECOSSparseMatrix::fromSparse(mat);

  EXPECT_EQ(result.nrows, 3);
  EXPECT_EQ(result.ncols, 3);
  EXPECT_EQ(result.nnz, 5);

  // Expected CSC:
  // Column 0: [1.0, 4.0] at rows [0, 2]
  // Column 1: [3.0] at row [1]
  // Column 2: [2.0, 5.0] at rows [0, 2]
  ASSERT_EQ(result.data.size(), 5);
  EXPECT_DOUBLE_EQ(result.data[0], 1.0);
  EXPECT_DOUBLE_EQ(result.data[1], 4.0);
  EXPECT_DOUBLE_EQ(result.data[2], 3.0);
  EXPECT_DOUBLE_EQ(result.data[3], 2.0);
  EXPECT_DOUBLE_EQ(result.data[4], 5.0);

  ASSERT_EQ(result.row_indices.size(), 5);
  EXPECT_EQ(result.row_indices[0], 0);
  EXPECT_EQ(result.row_indices[1], 2);
  EXPECT_EQ(result.row_indices[2], 1);
  EXPECT_EQ(result.row_indices[3], 0);
  EXPECT_EQ(result.row_indices[4], 2);

  ASSERT_EQ(result.col_ptrs.size(), 4);
  EXPECT_EQ(result.col_ptrs[0], 0);
  EXPECT_EQ(result.col_ptrs[1], 2);
  EXPECT_EQ(result.col_ptrs[2], 3);
  EXPECT_EQ(result.col_ptrs[3], 5);
}

TEST(ECOSSparseMatrix, FromDenseAndFromSparseProduceSameResult)
{
  Eigen::MatrixXd dense{3, 3};
  dense << 1.0, 0.0, 2.0,
           0.0, 3.0, 0.0,
           4.0, 0.0, 5.0;

  Eigen::SparseMatrix<double> sparse{3, 3};
  sparse.insert(0, 0) = 1.0;
  sparse.insert(2, 0) = 4.0;
  sparse.insert(1, 1) = 3.0;
  sparse.insert(0, 2) = 2.0;
  sparse.insert(2, 2) = 5.0;
  sparse.makeCompressed();

  auto result_dense = ECOSSparseMatrix::fromDense(dense);
  auto result_sparse = ECOSSparseMatrix::fromSparse(sparse);

  EXPECT_TRUE(csrEqual(result_dense, result_sparse));
}

TEST(ECOSSparseMatrix, CopyConstructorWorks)
{
  Eigen::MatrixXd mat = Eigen::MatrixXd::Identity(3, 3);
  auto original = ECOSSparseMatrix::fromDense(mat);
  auto copy{original};

  EXPECT_TRUE(csrEqual(original, copy));
}

TEST(ECOSSparseMatrix, MoveConstructorWorks)
{
  Eigen::MatrixXd mat = Eigen::MatrixXd::Identity(3, 3);
  auto original = ECOSSparseMatrix::fromDense(mat);
  auto expected = original;
  auto moved{std::move(original)};

  EXPECT_TRUE(csrEqual(expected, moved));
}
