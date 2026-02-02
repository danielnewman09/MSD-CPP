// Ticket: 0035b3_ecos_problem_construction
// Ticket: 0035d1_ecos_socp_reformulation
// Ticket: 0035d6_ecos_problem_builder_test_realignment
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md
// Design: docs/designs/0035d1_ecos_socp_reformulation/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSProblemBuilder.hpp"
#include <Eigen/Dense>
#include <cmath>

using namespace msd_sim;

// AC1: buildECOSProblem produces correct G matrix dimensions for single contact
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

  // SOCP auxiliary-variable formulation: x = [λ (3C); y (3C); t (1)]
  // For C=1: x dimension is 3+3+1 = 7
  // G matrix: (6C+2) × (6C+1) → 8 × 7
  // nnz: 6C+2 = 8 (friction block 3, y-diagonal 3, t-column 2)
  EXPECT_EQ(data.G_.nrows, 8);
  EXPECT_EQ(data.G_.ncols, 7);
  EXPECT_EQ(data.G_.nnz, 8);

  // Verify key structure elements in CSC format
  // Lambda columns (0-2): friction cone entries
  EXPECT_EQ(data.G_.col_ptrs[0], 0);
  EXPECT_EQ(data.G_.col_ptrs[1], 1);
  EXPECT_EQ(data.G_.col_ptrs[2], 2);
  EXPECT_EQ(data.G_.col_ptrs[3], 3);

  // Friction block: row 5, 6, 7 (after epigraph rows 0-4)
  EXPECT_EQ(data.G_.row_indices[0], 5);  // Friction row for λ_n
  EXPECT_EQ(data.G_.row_indices[1], 6);  // Friction row for λ_t1
  EXPECT_EQ(data.G_.row_indices[2], 7);  // Friction row for λ_t2

  // Friction values: [-μ, -1, -1]
  EXPECT_NEAR(data.G_.data[0], -0.5, 1e-10);
  EXPECT_NEAR(data.G_.data[1], -1.0, 1e-10);
  EXPECT_NEAR(data.G_.data[2], -1.0, 1e-10);

  // y columns (3-5): epigraph diagonal entries (rows 1-3)
  EXPECT_EQ(data.G_.row_indices[3], 1);
  EXPECT_EQ(data.G_.row_indices[4], 2);
  EXPECT_EQ(data.G_.row_indices[5], 3);
  EXPECT_NEAR(data.G_.data[3], -2.0, 1e-10);
  EXPECT_NEAR(data.G_.data[4], -2.0, 1e-10);
  EXPECT_NEAR(data.G_.data[5], -2.0, 1e-10);

  // t column (6): two epigraph entries (rows 0 and 4)
  EXPECT_EQ(data.G_.row_indices[6], 0);
  EXPECT_EQ(data.G_.row_indices[7], 4);
  EXPECT_NEAR(data.G_.data[6], -1.0, 1e-10);
  EXPECT_NEAR(data.G_.data[7], -1.0, 1e-10);
}

// AC2: buildECOSProblem produces correct G matrix dimensions for multi-contact (2+ contacts)
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

  // SOCP auxiliary-variable formulation: x = [λ (6); y (6); t (1)]
  // For C=2: x dimension is 6+6+1 = 13
  // G matrix: (6C+2) × (6C+1) → 14 × 13
  // nnz: 6C+2 = 14
  EXPECT_EQ(data.G_.nrows, 14);
  EXPECT_EQ(data.G_.ncols, 13);
  EXPECT_EQ(data.G_.nnz, 14);

  // Friction block entries (lambda columns 0-5, in friction cone rows)
  // Contact 0: rows 8, 9, 10 (after epigraph 0-7)
  // Contact 1: rows 11, 12, 13
  EXPECT_NEAR(data.G_.data[0], -0.3, 1e-10);  // Contact 0 normal
  EXPECT_NEAR(data.G_.data[1], -1.0, 1e-10);  // Contact 0 tangent1
  EXPECT_NEAR(data.G_.data[2], -1.0, 1e-10);  // Contact 0 tangent2
  EXPECT_NEAR(data.G_.data[3], -0.8, 1e-10);  // Contact 1 normal
  EXPECT_NEAR(data.G_.data[4], -1.0, 1e-10);  // Contact 1 tangent1
  EXPECT_NEAR(data.G_.data[5], -1.0, 1e-10);  // Contact 1 tangent2

  // y block entries (columns 6-11, diagonal -2 in epigraph rows 1-6)
  for (int i = 6; i < 12; ++i)
  {
    EXPECT_NEAR(data.G_.data[static_cast<size_t>(i)], -2.0, 1e-10);
  }

  // t column entries (column 12, rows 0 and 7)
  EXPECT_NEAR(data.G_.data[12], -1.0, 1e-10);
  EXPECT_NEAR(data.G_.data[13], -1.0, 1e-10);
}

// AC3: G matrix dimensions scale correctly with contact count
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

  // SOCP auxiliary-variable formulation: x = [λ (9); y (9); t (1)]
  // For C=3: x dimension is 9+9+1 = 19
  // G matrix: (6C+2) × (6C+1) → 20 × 19
  // nnz: 6C+2 = 20
  EXPECT_EQ(data.G_.nrows, 20);
  EXPECT_EQ(data.G_.ncols, 19);
  EXPECT_EQ(data.G_.nnz, 20);
  EXPECT_EQ(data.num_variables_, 19);
  EXPECT_EQ(data.num_cones_, 4);  // 1 epigraph + 3 friction
}

// AC4: h vector contains non-zero epigraph RHS terms
TEST(ECOSProblemBuilderTest, HVectorAllZeros)
{
  const int numContacts = 2;
  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);
  coneSpec.setFriction(1, 0.3, 3);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(6);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // h vector size: 6C+2 = 14
  // Structure: [1; -2d (6 elements); -1; 0 (6 friction zeros)]
  EXPECT_EQ(data.h_.size(), 14u);

  // Epigraph entries are non-zero
  EXPECT_NEAR(data.h_[0], 1.0, 1e-10);   // Row 0: t+1 bound
  EXPECT_NEAR(data.h_[7], -1.0, 1e-10);  // Row 7 (3C+1): t-1 term

  // Middle 6 elements are -2d = -2·L⁻¹b (non-zero for b=ones)
  // For identity A (L=I), d = b, so -2d = -2·ones
  for (size_t i = 1; i <= 6; ++i)
  {
    EXPECT_NEAR(data.h_[i], -2.0, 1e-10);
  }

  // Friction cone RHS is all zeros
  for (size_t i = 8; i < 14; ++i)
  {
    EXPECT_NEAR(data.h_[i], 0.0, 1e-10);
  }
}

// AC5: c vector minimizes epigraph variable t
TEST(ECOSProblemBuilderTest, CVectorAllZeros)
{
  const int numContacts = 1;
  FrictionConeSpec coneSpec{numContacts};
  coneSpec.setFriction(0, 0.5, 0);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(3);

  ECOSData data = ECOSProblemBuilder::build(A, b, coneSpec);

  // c vector size: 6C+1 = 7
  // Structure: [0 (λ); 0 (y); 1 (t)]
  EXPECT_EQ(data.c_.size(), 7u);

  // All zeros except last element
  for (size_t i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(data.c_[i], 0.0, 1e-10);
  }

  // Last element is 1.0 (minimize t)
  EXPECT_NEAR(data.c_[6], 1.0, 1e-10);
}

// AC6: Cone sizes include epigraph cone
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

  // Cone structure: [3C+2, 3, 3, 3, 3]
  // First cone is epigraph (size 3C+2 = 14)
  // Followed by C=4 friction cones (size 3 each)
  EXPECT_EQ(data.cone_sizes_.size(), 5u);
  EXPECT_EQ(data.cone_sizes_[0], 14);  // Epigraph cone
  EXPECT_EQ(data.cone_sizes_[1], 3);   // Friction cone 0
  EXPECT_EQ(data.cone_sizes_[2], 3);   // Friction cone 1
  EXPECT_EQ(data.cone_sizes_[3], 3);   // Friction cone 2
  EXPECT_EQ(data.cone_sizes_[4], 3);   // Friction cone 3
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

  // Lambda columns 0, 3, 6 contain normal friction coefficients
  // (first entry in each friction cone triplet)
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

  // Lambda column 0 (normal): friction coefficient should be 0.0
  EXPECT_NEAR(data.G_.data[0], 0.0, 1e-10);
  // Tangent columns still have -1.0
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

// SOCP G matrix structure validation (replaces BlockDiagonalStructure test)
// The SOCP G matrix is NOT block-diagonal due to auxiliary variables y and t
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
  const int nrows = static_cast<int>(data.G_.nrows);
  const int ncols = static_cast<int>(data.G_.ncols);
  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Zero(nrows, ncols);

  for (idxint col = 0; col < data.G_.ncols; ++col)
  {
    for (idxint idx = data.G_.col_ptrs[static_cast<size_t>(col)];
         idx < data.G_.col_ptrs[static_cast<size_t>(col + 1)]; ++idx)
    {
      idxint row = data.G_.row_indices[static_cast<size_t>(idx)];
      G_dense(row, col) = data.G_.data[static_cast<size_t>(idx)];
    }
  }

  // SOCP structure validation:
  // G is (20 × 19) = (6C+2) × (6C+1) for C=3
  // Structure: [λ cols (0-8); y cols (9-17); t col (18)]

  // Lambda columns (0-8): should only have entries in friction rows (11-19)
  for (int col = 0; col < 9; ++col)
  {
    for (int row = 0; row < nrows; ++row)
    {
      if (row < 11)  // Epigraph rows (0-10)
      {
        EXPECT_NEAR(G_dense(row, col), 0.0, 1e-10);
      }
    }
  }

  // y columns (9-17): should only have entries in epigraph middle rows (1-9)
  for (int col = 9; col < 18; ++col)
  {
    for (int row = 0; row < nrows; ++row)
    {
      const int y_idx = col - 9;
      if (row == y_idx + 1)  // Diagonal entry in epigraph block
      {
        EXPECT_NEAR(G_dense(row, col), -2.0, 1e-10);
      }
      else
      {
        EXPECT_NEAR(G_dense(row, col), 0.0, 1e-10);
      }
    }
  }

  // t column (18): should have two entries (rows 0 and 10)
  EXPECT_NEAR(G_dense(0, 18), -1.0, 1e-10);    // Epigraph row 0
  EXPECT_NEAR(G_dense(10, 18), -1.0, 1e-10);   // Epigraph row 3C+1
  for (int row = 1; row < nrows; ++row)
  {
    if (row != 10)
    {
      EXPECT_NEAR(G_dense(row, 18), 0.0, 1e-10);
    }
  }

  // Friction block (rows 11-19, cols 0-8): block-diagonal structure
  for (int contact = 0; contact < numContacts; ++contact)
  {
    const int lambda_base = 3 * contact;
    const int fric_row_base = 11 + 3 * contact;

    EXPECT_NEAR(G_dense(fric_row_base, lambda_base), -0.5, 1e-10);         // -μ
    EXPECT_NEAR(G_dense(fric_row_base + 1, lambda_base + 1), -1.0, 1e-10);  // -1
    EXPECT_NEAR(G_dense(fric_row_base + 2, lambda_base + 2), -1.0, 1e-10);  // -1
  }
}

// Verify CSC format correctness for SOCP matrix structure
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

  // SOCP structure: most columns have 1 nnz, t column has 2 nnz
  // Lambda columns (0-5): 1 entry each (friction)
  // y columns (6-11): 1 entry each (epigraph diagonal)
  // t column (12): 2 entries (epigraph rows 0 and 7)

  for (idxint col = 0; col < 12; ++col)
  {
    const idxint nnz_in_col = data.G_.col_ptrs[static_cast<size_t>(col + 1)]
                              - data.G_.col_ptrs[static_cast<size_t>(col)];
    EXPECT_EQ(nnz_in_col, 1);
  }

  // t column (last column)
  const idxint nnz_in_t = data.G_.col_ptrs[static_cast<size_t>(data.G_.ncols)]
                          - data.G_.col_ptrs[static_cast<size_t>(data.G_.ncols - 1)];
  EXPECT_EQ(nnz_in_t, 2);

  // Verify data/row_indices arrays have correct sizes
  EXPECT_EQ(data.G_.data.size(), static_cast<size_t>(data.G_.nnz));
  EXPECT_EQ(data.G_.row_indices.size(), static_cast<size_t>(data.G_.nnz));
}
