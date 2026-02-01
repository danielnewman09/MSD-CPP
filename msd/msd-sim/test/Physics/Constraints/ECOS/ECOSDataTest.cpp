// Ticket: 0035b2_ecos_data_wrapper
// Design: docs/designs/0035b2_ecos_data_wrapper/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp"
#include <Eigen/Dense>

using namespace msd_sim;

// AC1: ECOSData constructor initializes dimensions and reserves vector storage
TEST(ECOSData, ConstructorInitializesDimensions)
{
  ECOSData data{9, 3};

  EXPECT_EQ(data.num_variables_, 9);
  EXPECT_EQ(data.num_cones_, 3);
}

TEST(ECOSData, ConstructorReservesVectorStorage)
{
  ECOSData data{9, 3};

  EXPECT_GE(data.h_.capacity(), 9);
  EXPECT_GE(data.c_.capacity(), 9);
  EXPECT_GE(data.cone_sizes_.capacity(), 3);
}

TEST(ECOSData, ConstructorLeavesWorkspaceNull)
{
  ECOSData data{9, 3};

  EXPECT_EQ(data.workspace_.get(), nullptr);
  EXPECT_FALSE(data.isSetup());
}

// AC2: setup() successfully calls ECOS_setup() with valid data
TEST(ECOSData, SetupSucceedsWithValidData)
{
  ECOSData data{3, 1};

  // Populate G matrix (3x3 identity)
  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);

  // Populate h vector (3 elements)
  data.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};

  // Populate c vector (3 elements, zero objective)
  data.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};

  // Populate cone sizes (1 cone of size 3)
  data.cone_sizes_ = std::vector<idxint>{3};

  // Setup should succeed
  EXPECT_NO_THROW(data.setup());
  EXPECT_TRUE(data.isSetup());
  EXPECT_NE(data.workspace_.get(), nullptr);
}

TEST(ECOSData, SetupThrowsWhenGMatrixEmpty)
{
  ECOSData data{3, 1};

  // Do not populate G (nnz == 0)
  data.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};
  data.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
  data.cone_sizes_ = std::vector<idxint>{3};

  EXPECT_THROW(data.setup(), std::runtime_error);
  EXPECT_FALSE(data.isSetup());
}

TEST(ECOSData, SetupThrowsWhenHSizeMismatch)
{
  ECOSData data{3, 1};

  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);

  // Wrong size for h
  data.h_ = std::vector<pfloat>{1.0, 1.0};  // Should be 3 elements
  data.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
  data.cone_sizes_ = std::vector<idxint>{3};

  EXPECT_THROW(data.setup(), std::runtime_error);
  EXPECT_FALSE(data.isSetup());
}

TEST(ECOSData, SetupThrowsWhenCSizeMismatch)
{
  ECOSData data{3, 1};

  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);

  data.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};
  // Wrong size for c
  data.c_ = std::vector<pfloat>{0.0};  // Should be 3 elements
  data.cone_sizes_ = std::vector<idxint>{3};

  EXPECT_THROW(data.setup(), std::runtime_error);
  EXPECT_FALSE(data.isSetup());
}

TEST(ECOSData, SetupThrowsWhenConeSizesMismatch)
{
  ECOSData data{3, 1};

  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);

  data.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};
  data.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
  // Wrong size for cone_sizes
  data.cone_sizes_ = std::vector<idxint>{};  // Should be 1 element

  EXPECT_THROW(data.setup(), std::runtime_error);
  EXPECT_FALSE(data.isSetup());
}

// AC3: Destructor calls ECOS_cleanup() (verified via workspace becoming null)
TEST(ECOSData, DestructorCleansUpWorkspace)
{
  pwork* raw_ptr = nullptr;

  {
    ECOSData data{3, 1};

    Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
    data.G_ = ECOSSparseMatrix::fromDense(G_dense);
    data.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};
    data.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
    data.cone_sizes_ = std::vector<idxint>{3};

    data.setup();
    ASSERT_TRUE(data.isSetup());

    // Store raw pointer (for observational purposes only, not for access)
    raw_ptr = data.workspace_.get();
    ASSERT_NE(raw_ptr, nullptr);

    // ECOSData goes out of scope, destructor should cleanup
  }

  // After destruction, we cannot safely access raw_ptr
  // But AddressSanitizer will detect any memory leaks
  // This test validates that no leak occurs (ASAN will fail if leak exists)
}

// AC4: Move constructor transfers workspace ownership correctly
TEST(ECOSData, MoveConstructorTransfersOwnership)
{
  ECOSData data{3, 1};

  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);
  data.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};
  data.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
  data.cone_sizes_ = std::vector<idxint>{3};
  data.setup();

  pwork* original_ptr = data.workspace_.get();
  ASSERT_NE(original_ptr, nullptr);

  // Move construct
  ECOSData moved{std::move(data)};

  // Target owns workspace
  EXPECT_EQ(moved.workspace_.get(), original_ptr);
  EXPECT_TRUE(moved.isSetup());

  // Source workspace nullified
  EXPECT_EQ(data.workspace_.get(), nullptr);
  EXPECT_FALSE(data.isSetup());
}

TEST(ECOSData, MoveConstructorTransfersDimensions)
{
  ECOSData data{9, 3};

  ECOSData moved{std::move(data)};

  EXPECT_EQ(moved.num_variables_, 9);
  EXPECT_EQ(moved.num_cones_, 3);
}

TEST(ECOSData, MoveConstructorTransfersVectors)
{
  ECOSData data{3, 1};

  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);
  data.h_ = std::vector<pfloat>{1.0, 2.0, 3.0};
  data.c_ = std::vector<pfloat>{4.0, 5.0, 6.0};
  data.cone_sizes_ = std::vector<idxint>{3};

  ECOSData moved{std::move(data)};

  ASSERT_EQ(moved.h_.size(), 3);
  EXPECT_DOUBLE_EQ(moved.h_[0], 1.0);
  EXPECT_DOUBLE_EQ(moved.h_[1], 2.0);
  EXPECT_DOUBLE_EQ(moved.h_[2], 3.0);

  ASSERT_EQ(moved.c_.size(), 3);
  EXPECT_DOUBLE_EQ(moved.c_[0], 4.0);
  EXPECT_DOUBLE_EQ(moved.c_[1], 5.0);
  EXPECT_DOUBLE_EQ(moved.c_[2], 6.0);

  ASSERT_EQ(moved.cone_sizes_.size(), 1);
  EXPECT_EQ(moved.cone_sizes_[0], 3);
}

// AC5: Move assignment cleans up existing workspace before transfer
TEST(ECOSData, MoveAssignmentCleansUpExistingWorkspace)
{
  // Create first ECOSData with workspace
  ECOSData data1{3, 1};
  Eigen::MatrixXd G_dense1 = Eigen::MatrixXd::Identity(3, 3);
  data1.G_ = ECOSSparseMatrix::fromDense(G_dense1);
  data1.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};
  data1.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
  data1.cone_sizes_ = std::vector<idxint>{3};
  data1.setup();

  pwork* data1_ptr = data1.workspace_.get();
  ASSERT_NE(data1_ptr, nullptr);

  // Create second ECOSData with workspace
  ECOSData data2{3, 1};
  Eigen::MatrixXd G_dense2 = Eigen::MatrixXd::Identity(3, 3);
  data2.G_ = ECOSSparseMatrix::fromDense(G_dense2);
  data2.h_ = std::vector<pfloat>{2.0, 2.0, 2.0};
  data2.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
  data2.cone_sizes_ = std::vector<idxint>{3};
  data2.setup();

  pwork* data2_ptr = data2.workspace_.get();
  ASSERT_NE(data2_ptr, nullptr);
  ASSERT_NE(data2_ptr, data1_ptr);  // Different workspaces

  // Move assign data2 to data1 (data1's old workspace should be cleaned)
  data1 = std::move(data2);

  // data1 now owns data2's workspace
  EXPECT_EQ(data1.workspace_.get(), data2_ptr);
  EXPECT_TRUE(data1.isSetup());

  // data2's workspace nullified
  EXPECT_EQ(data2.workspace_.get(), nullptr);
  EXPECT_FALSE(data2.isSetup());

  // data1's old workspace was cleaned (ASAN will detect if not freed)
}

TEST(ECOSData, MoveAssignmentTransfersDimensions)
{
  ECOSData data1{3, 1};
  ECOSData data2{9, 3};

  data1 = std::move(data2);

  EXPECT_EQ(data1.num_variables_, 9);
  EXPECT_EQ(data1.num_cones_, 3);
}

// AC6: cleanup() is safe to call when workspace is null (idempotent)
TEST(ECOSData, CleanupIsIdempotentWhenNull)
{
  ECOSData data{3, 1};

  ASSERT_FALSE(data.isSetup());

  // Should not throw when workspace is null
  EXPECT_NO_THROW(data.cleanup());
  EXPECT_FALSE(data.isSetup());
}

TEST(ECOSData, CleanupNullifiesWorkspace)
{
  ECOSData data{3, 1};

  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);
  data.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};
  data.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
  data.cone_sizes_ = std::vector<idxint>{3};
  data.setup();

  ASSERT_TRUE(data.isSetup());

  // Cleanup should nullify workspace
  data.cleanup();
  EXPECT_FALSE(data.isSetup());
  EXPECT_EQ(data.workspace_.get(), nullptr);
}

TEST(ECOSData, CleanupIsIdempotentAfterCleanup)
{
  ECOSData data{3, 1};

  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);
  data.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};
  data.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
  data.cone_sizes_ = std::vector<idxint>{3};
  data.setup();
  data.cleanup();

  ASSERT_FALSE(data.isSetup());

  // Second cleanup should be safe
  EXPECT_NO_THROW(data.cleanup());
  EXPECT_FALSE(data.isSetup());
}

// AC7: Copy construction is a compile error (= delete)
// This is enforced at compile time, no runtime test needed
// Uncommenting the following line should produce a compiler error:
// TEST(ECOSData, CopyConstructorDeleted) {
//   ECOSData data{3, 1};
//   ECOSData copy{data};  // Compile error: copy constructor deleted
// }

// AC8: All existing tests pass (zero regressions)
// This is validated by running the full test suite after implementation

// Additional tests for robustness

TEST(ECOSData, SetupWithMultipleCones)
{
  ECOSData data{9, 3};

  // 9x9 identity matrix
  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(9, 9);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);
  data.h_ = std::vector<pfloat>(9, 1.0);
  data.c_ = std::vector<pfloat>(9, 0.0);
  data.cone_sizes_ = std::vector<idxint>{3, 3, 3};

  EXPECT_NO_THROW(data.setup());
  EXPECT_TRUE(data.isSetup());
}

TEST(ECOSData, WorkspaceAccessViaGet)
{
  ECOSData data{3, 1};

  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);
  data.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};
  data.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
  data.cone_sizes_ = std::vector<idxint>{3};
  data.setup();

  // Should be able to access workspace via get()
  pwork* w = data.workspace_.get();
  ASSERT_NE(w, nullptr);

  // Verify ECOS workspace structure is valid
  EXPECT_NE(w->stgs, nullptr);  // Settings should exist
}

TEST(ECOSData, WorkspaceAccessViaArrow)
{
  ECOSData data{3, 1};

  Eigen::MatrixXd G_dense = Eigen::MatrixXd::Identity(3, 3);
  data.G_ = ECOSSparseMatrix::fromDense(G_dense);
  data.h_ = std::vector<pfloat>{1.0, 1.0, 1.0};
  data.c_ = std::vector<pfloat>{0.0, 0.0, 0.0};
  data.cone_sizes_ = std::vector<idxint>{3};
  data.setup();

  // Should be able to access workspace members via ->
  ASSERT_TRUE(data.isSetup());
  EXPECT_NE(data.workspace_->stgs, nullptr);
}

TEST(ECOSData, SetupWithSparseMatrix)
{
  ECOSData data{6, 2};

  // Create sparse matrix with specific pattern
  Eigen::MatrixXd G_dense(6, 6);
  G_dense << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

  data.G_ = ECOSSparseMatrix::fromDense(G_dense);
  data.h_ = std::vector<pfloat>(6, 1.0);
  data.c_ = std::vector<pfloat>(6, 0.0);
  data.cone_sizes_ = std::vector<idxint>{3, 3};

  EXPECT_NO_THROW(data.setup());
  EXPECT_TRUE(data.isSetup());
}
