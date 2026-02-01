// Ticket: 0035b3_ecos_problem_construction
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp"
#include <Eigen/Dense>
#include <cmath>

using namespace msd_sim;

// AC1: buildECOSProblem produces correct G matrix for single contact (validated against hand computation)
TEST(ECOSProblemBuilderTest, SingleContactGMatrix)
{
  // Setup: Single contact with friction coefficient μ = 0.5
  const int numContacts = 1;
  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);

  // Build trivial A matrix and b vector (3×3 for single contact)
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(3);

  // Build ECOS problem
  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // Verify G matrix dimensions
  EXPECT_EQ(data.G_.nrows, 3);
  EXPECT_EQ(data.G_.ncols, 3);
  EXPECT_EQ(data.G_.nnz, 3);

  // G matrix should be diagonal with [-μ, -1, -1]
  // CSC format: data[i] at row_indices[i] for column col_ptrs[j]:col_ptrs[j+1]

  // Column 0: G[0,0] = -0.5
  EXPECT_EQ(data.G_.col_ptrs[0], 0);
  EXPECT_EQ(data.G_.col_ptrs[1], 1);
  EXPECT_EQ(data.G_.row_indices[0], 0);
  EXPECT_NEAR(data.G_.data[0], -0.5, 1e-10);

  // Column 1: G[1,1] = -1.0
  EXPECT_EQ(data.G_.col_ptrs[2], 2);
  EXPECT_EQ(data.G_.row_indices[1], 1);
  EXPECT_NEAR(data.G_.data[1], -1.0, 1e-10);

  // Column 2: G[2,2] = -1.0
  EXPECT_EQ(data.G_.col_ptrs[3], 3);
  EXPECT_EQ(data.G_.row_indices[2], 2);
  EXPECT_NEAR(data.G_.data[2], -1.0, 1e-10);
}

// AC2: buildECOSProblem produces correct G matrix for multi-contact (2+ contacts)
TEST(ECOSProblemBuilderTest, MultiContactGMatrix)
{
  // Setup: Two contacts with different friction coefficients
  const int numContacts = 2;
  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.3, 0);  // Contact 0: μ = 0.3
  coneSpec.setFriction(1, 0.8, 3);  // Contact 1: μ = 0.8

  // Build trivial A matrix and b vector (6×6 for two contacts)
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(6);

  // Build ECOS problem
  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // Verify G matrix dimensions
  EXPECT_EQ(data.G_.nrows, 6);
  EXPECT_EQ(data.G_.ncols, 6);
  EXPECT_EQ(data.G_.nnz, 6);  // Block-diagonal: 3 non-zeros per contact

  // Contact 0: indices [0, 1, 2] → [-0.3, -1, -1]
  EXPECT_NEAR(data.G_.data[0], -0.3, 1e-10);
  EXPECT_NEAR(data.G_.data[1], -1.0, 1e-10);
  EXPECT_NEAR(data.G_.data[2], -1.0, 1e-10);

  // Contact 1: indices [3, 4, 5] → [-0.8, -1, -1]
  EXPECT_NEAR(data.G_.data[3], -0.8, 1e-10);
  EXPECT_NEAR(data.G_.data[4], -1.0, 1e-10);
  EXPECT_NEAR(data.G_.data[5], -1.0, 1e-10);

  // Verify block-diagonal structure (each contact independent)
  for (size_t i = 0; i < 6; ++i)
  {
    EXPECT_EQ(data.G_.row_indices[i], static_cast<idxint>(i));  // Diagonal structure
  }
}

// AC3: G matrix dimensions match 3C x 3C
TEST(ECOSProblemBuilderTest, GMatrixDimensions)
{
  const int numContacts = 3;
  FrictionConeSpec coneSpec{numContacts};
  for (int i = 0; i < numContacts; ++i)
  {
    coneSpec.setFriction(i, 0.5, 3 * i);
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(9, 9);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(9);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // Verify dimensions for 3 contacts
  EXPECT_EQ(data.G_.nrows, 9);
  EXPECT_EQ(data.G_.ncols, 9);
  EXPECT_EQ(data.G_.nnz, 9);
  EXPECT_EQ(data.num_variables_, 9);
  EXPECT_EQ(data.num_cones_, 3);
}

// AC4: h vector is correct for friction cone formulation
TEST(ECOSProblemBuilderTest, HVectorAllZeros)
{
  const int numContacts = 2;
  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);
  coneSpec.setFriction(1, 0.3, 3);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(6);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // h vector should be all zeros for standard friction cone
  EXPECT_EQ(data.h_.size(), 6u);
  for (size_t i = 0; i < data.h_.size(); ++i)
  {
    EXPECT_NEAR(data.h_[i], 0.0, 1e-10);
  }
}

// Linear objective c is zero for LCP formulation
TEST(ECOSProblemBuilderTest, CVectorAllZeros)
{
  const int numContacts = 1;
  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(3);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // c vector should be all zeros (LCP formulation, not minimizing a linear objective)
  EXPECT_EQ(data.c_.size(), 3u);
  for (size_t i = 0; i < data.c_.size(); ++i)
  {
    EXPECT_NEAR(data.c_[i], 0.0, 1e-10);
  }
}

// Cone size array is [3, 3, ..., 3] for C contacts
TEST(ECOSProblemBuilderTest, ConeSizes)
{
  const int numContacts = 4;
  FrictionConeSpec coneSpec{numContacts};
  for (int i = 0; i < numContacts; ++i)
  {
    coneSpec.setFriction(i, 0.5, 3 * i);
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(12, 12);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(12);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // All cones should have size 3
  EXPECT_EQ(data.cone_sizes_.size(), 4u);
  for (size_t i = 0; i < data.cone_sizes_.size(); ++i)
  {
    EXPECT_EQ(data.cone_sizes_[i], 3);
  }
}

// Each contact can have different friction coefficient
TEST(ECOSProblemBuilderTest, DifferentMuValues)
{
  const int numContacts = 3;
  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.1, 0);
  coneSpec.setFriction(1, 0.5, 3);
  coneSpec.setFriction(2, 0.9, 6);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(9, 9);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(9);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // Each contact's normal component should have -μ_i
  EXPECT_NEAR(data.G_.data[0], -0.1, 1e-10);  // Contact 0: -0.1
  EXPECT_NEAR(data.G_.data[3], -0.5, 1e-10);  // Contact 1: -0.5
  EXPECT_NEAR(data.G_.data[6], -0.9, 1e-10);  // Contact 2: -0.9
}

// Zero friction coefficient produces degenerate cone
TEST(ECOSProblemBuilderTest, ZeroMuDegenerateCone)
{
  const int numContacts = 1;
  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.0, 0);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(3);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // G[0,0] should be 0.0 (degenerate cone forces λ_t = 0)
  EXPECT_NEAR(data.G_.data[0], 0.0, 1e-10);
  EXPECT_NEAR(data.G_.data[1], -1.0, 1e-10);
  EXPECT_NEAR(data.G_.data[2], -1.0, 1e-10);
}

// AC7: Invalid inputs produce descriptive exceptions
TEST(ECOSProblemBuilderTest, NonSquareAMatrixThrows)
{
  FrictionConeSpec coneSpec{1};
  coneSpec.setFriction(0, 0.5, 0);

  Eigen::MatrixXd A(3, 4);  // Not square
  Eigen::VectorXd b = Eigen::VectorXd::Ones(3);

  EXPECT_THROW(ECOSProblemBuilder::build(A, b, coneSpec), std::invalid_argument);
}

TEST(ECOSProblemBuilderTest, ADimensionMismatchWithB)
{
  FrictionConeSpec coneSpec{1};
  coneSpec.setFriction(0, 0.5, 0);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(4);  // Wrong size

  EXPECT_THROW(ECOSProblemBuilder::build(A, b, coneSpec), std::invalid_argument);
}

TEST(ECOSProblemBuilderTest, ADimensionMismatchWithNumContacts)
{
  FrictionConeSpec coneSpec{2};  // 2 contacts → expect 6×6 matrix
  coneSpec.setFriction(0, 0.5, 0);
  coneSpec.setFriction(1, 0.3, 3);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);  // Wrong size (should be 6×6)
  Eigen::VectorXd b = Eigen::VectorXd::Ones(3);

  EXPECT_THROW(ECOSProblemBuilder::build(A, b, coneSpec), std::invalid_argument);
}

TEST(ECOSProblemBuilderTest, ZeroContactsThrows)
{
  FrictionConeSpec coneSpec{0};  // Invalid: no contacts

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(3);

  EXPECT_THROW(ECOSProblemBuilder::build(A, b, coneSpec), std::invalid_argument);
}

// Verify truly block-diagonal structure
TEST(ECOSProblemBuilderTest, BlockDiagonalStructure)
{
  const int numContacts = 3;
  FrictionConeSpec coneSpec{numContacts};
  for (int i = 0; i < numContacts; ++i)
  {
    coneSpec.setFriction(i, 0.5, 3 * i);
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(9, 9);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(9);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // Reconstruct dense G matrix from CSC format
  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Zero(9, 9);
  for (idxint col = 0; col < data.G_.ncols; ++col)
  {
    for (idxint idx = data.G_.col_ptrs[static_cast<size_t>(col)];
         idx < data.G_.col_ptrs[static_cast<size_t>(col + 1)]; ++idx)
    {
      idxint row = data.G_.row_indices[static_cast<size_t>(idx)];
      G_dense(row, col) = data.G_.data[static_cast<size_t>(idx)];
    }
  }

  // Verify block-diagonal structure: all off-diagonal blocks should be zero
  for (int i = 0; i < 9; ++i)
  {
    for (int j = 0; j < 9; ++j)
    {
      const int block_i = i / 3;
      const int block_j = j / 3;

      if (block_i != block_j)
      {
        // Off-diagonal block: should be zero
        EXPECT_NEAR(G_dense(i, j), 0.0, 1e-10);
      }
    }
  }

  // Verify diagonal blocks are correct
  for (int contact = 0; contact < numContacts; ++contact)
  {
    const int base = 3 * contact;
    EXPECT_NEAR(G_dense(base, base), -0.5, 1e-10);         // -μ
    EXPECT_NEAR(G_dense(base + 1, base + 1), -1.0, 1e-10);  // -1
    EXPECT_NEAR(G_dense(base + 2, base + 2), -1.0, 1e-10);  // -1
  }
}

// Verify CSC format correctness
TEST(ECOSProblemBuilderTest, CSCFormatValidation)
{
  const int numContacts = 2;
  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.4, 0);
  coneSpec.setFriction(1, 0.6, 3);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(6);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // CSC format invariants
  EXPECT_EQ(data.G_.col_ptrs.size(), static_cast<size_t>(data.G_.ncols + 1));
  EXPECT_EQ(data.G_.col_ptrs[0], 0);
  EXPECT_EQ(data.G_.col_ptrs[static_cast<size_t>(data.G_.ncols)], data.G_.nnz);

  // Each column should have exactly 1 non-zero (diagonal matrix)
  for (idxint col = 0; col < data.G_.ncols; ++col)
  {
    const idxint nnz_in_col = data.G_.col_ptrs[static_cast<size_t>(col + 1)]
                              - data.G_.col_ptrs[static_cast<size_t>(col)];
    EXPECT_EQ(nnz_in_col, 1);
  }

  // Row indices should match column indices (diagonal)
  for (idxint col = 0; col < data.G_.ncols; ++col)
  {
    const idxint idx = data.G_.col_ptrs[static_cast<size_t>(col)];
    EXPECT_EQ(data.G_.row_indices[static_cast<size_t>(idx)], col);
  }
}
