// Ticket: 0035b_box_constrained_asm_solver
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp"

using namespace msd_sim;

TEST(FrictionConeSpec, DefaultConstructorCreatesEmptySpec)
{
  FrictionConeSpec spec{};

  EXPECT_EQ(spec.numContacts, 0);
  EXPECT_TRUE(spec.frictionCoefficients.empty());
  EXPECT_TRUE(spec.normalIndices.empty());
}

TEST(FrictionConeSpec, ConstructorThrowsForNegativeContacts)
{
  EXPECT_THROW(FrictionConeSpec{-1}, std::invalid_argument);
}

TEST(FrictionConeSpec, ConstructorReservesSpaceForContacts)
{
  FrictionConeSpec spec{5};

  EXPECT_EQ(spec.numContacts, 5);
  EXPECT_GE(spec.frictionCoefficients.capacity(), 5);
  EXPECT_GE(spec.normalIndices.capacity(), 5);
}

TEST(FrictionConeSpec, SetFrictionStoresCoefficient)
{
  FrictionConeSpec spec{3};
  spec.setFriction(0, 0.5, 0);
  spec.setFriction(1, 0.8, 3);
  spec.setFriction(2, 0.3, 6);

  ASSERT_EQ(spec.frictionCoefficients.size(), 3);
  EXPECT_DOUBLE_EQ(spec.frictionCoefficients[0], 0.5);
  EXPECT_DOUBLE_EQ(spec.frictionCoefficients[1], 0.8);
  EXPECT_DOUBLE_EQ(spec.frictionCoefficients[2], 0.3);

  ASSERT_EQ(spec.normalIndices.size(), 3);
  EXPECT_EQ(spec.normalIndices[0], 0);
  EXPECT_EQ(spec.normalIndices[1], 3);
  EXPECT_EQ(spec.normalIndices[2], 6);
}

TEST(FrictionConeSpec, SetFrictionThrowsForOutOfRangeIndex)
{
  FrictionConeSpec spec{3};

  EXPECT_THROW(spec.setFriction(-1, 0.5, 0), std::invalid_argument);
  EXPECT_THROW(spec.setFriction(3, 0.5, 0), std::invalid_argument);
  EXPECT_THROW(spec.setFriction(100, 0.5, 0), std::invalid_argument);
}

TEST(FrictionConeSpec, SetFrictionThrowsForNegativeFrictionCoefficient)
{
  FrictionConeSpec spec{3};

  EXPECT_THROW(spec.setFriction(0, -0.1, 0), std::invalid_argument);
  EXPECT_THROW(spec.setFriction(1, -1.0, 3), std::invalid_argument);
}

TEST(FrictionConeSpec, SetFrictionAcceptsZeroFrictionCoefficient)
{
  FrictionConeSpec spec{1};

  EXPECT_NO_THROW(spec.setFriction(0, 0.0, 0));
  ASSERT_EQ(spec.frictionCoefficients.size(), 1);
  EXPECT_DOUBLE_EQ(spec.frictionCoefficients[0], 0.0);
}

TEST(FrictionConeSpec, GetConeSizesReturnsEmptyForZeroContacts)
{
  FrictionConeSpec spec{0};

  auto coneSizes = spec.getConeSizes();

  EXPECT_TRUE(coneSizes.empty());
}

TEST(FrictionConeSpec, GetConeSizesReturnsAllThreesForSingleContact)
{
  FrictionConeSpec spec{1};
  spec.setFriction(0, 0.5, 0);

  auto coneSizes = spec.getConeSizes();

  ASSERT_EQ(coneSizes.size(), 1);
  EXPECT_EQ(coneSizes[0], 3);
}

TEST(FrictionConeSpec, GetConeSizesReturnsAllThreesForMultipleContacts)
{
  FrictionConeSpec spec{5};
  spec.setFriction(0, 0.5, 0);
  spec.setFriction(1, 0.8, 3);
  spec.setFriction(2, 0.3, 6);
  spec.setFriction(3, 0.7, 9);
  spec.setFriction(4, 0.4, 12);

  auto coneSizes = spec.getConeSizes();

  ASSERT_EQ(coneSizes.size(), 5);
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(coneSizes[i], 3);
  }
}

TEST(FrictionConeSpec, GetConeSizesReturnsCorrectCountWithoutSettingFriction)
{
  // Test that getConeSizes() works even if setFriction() not called
  FrictionConeSpec spec{3};

  auto coneSizes = spec.getConeSizes();

  ASSERT_EQ(coneSizes.size(), 3);
  EXPECT_EQ(coneSizes[0], 3);
  EXPECT_EQ(coneSizes[1], 3);
  EXPECT_EQ(coneSizes[2], 3);
}

TEST(FrictionConeSpec, CopyConstructorWorks)
{
  FrictionConeSpec original{2};
  original.setFriction(0, 0.5, 0);
  original.setFriction(1, 0.8, 3);

  FrictionConeSpec copy{original};

  EXPECT_EQ(copy.numContacts, 2);
  ASSERT_EQ(copy.frictionCoefficients.size(), 2);
  EXPECT_DOUBLE_EQ(copy.frictionCoefficients[0], 0.5);
  EXPECT_DOUBLE_EQ(copy.frictionCoefficients[1], 0.8);
  ASSERT_EQ(copy.normalIndices.size(), 2);
  EXPECT_EQ(copy.normalIndices[0], 0);
  EXPECT_EQ(copy.normalIndices[1], 3);
}

TEST(FrictionConeSpec, MoveConstructorWorks)
{
  FrictionConeSpec original{2};
  original.setFriction(0, 0.5, 0);
  original.setFriction(1, 0.8, 3);

  FrictionConeSpec moved{std::move(original)};

  EXPECT_EQ(moved.numContacts, 2);
  ASSERT_EQ(moved.frictionCoefficients.size(), 2);
  EXPECT_DOUBLE_EQ(moved.frictionCoefficients[0], 0.5);
  EXPECT_DOUBLE_EQ(moved.frictionCoefficients[1], 0.8);
  ASSERT_EQ(moved.normalIndices.size(), 2);
  EXPECT_EQ(moved.normalIndices[0], 0);
  EXPECT_EQ(moved.normalIndices[1], 3);
}

TEST(FrictionConeSpec, NormalIndicesAreIndependent)
{
  FrictionConeSpec spec{3};

  // Normal indices can be arbitrary (not necessarily 3*i)
  spec.setFriction(0, 0.5, 7);
  spec.setFriction(1, 0.8, 2);
  spec.setFriction(2, 0.3, 15);

  ASSERT_EQ(spec.normalIndices.size(), 3);
  EXPECT_EQ(spec.normalIndices[0], 7);
  EXPECT_EQ(spec.normalIndices[1], 2);
  EXPECT_EQ(spec.normalIndices[2], 15);
}
