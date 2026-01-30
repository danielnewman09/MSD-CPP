// Ticket: 0032a_two_body_constraint_infrastructure
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include <gtest/gtest.h>
#include "msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp"
#include "msd-sim/src/Physics/Constraints/ContactConstraint.hpp"
#include "msd-sim/src/Environment/Coordinate.hpp"

using namespace msd_sim;

// ============================================================================
// TwoBodyConstraint Tests
// ============================================================================

TEST(TwoBodyConstraintTest, BodyIndexAccessors_0032a)
{
  // Test: getBodyAIndex() and getBodyBIndex() return constructor values
  size_t bodyAIndex = 3;
  size_t bodyBIndex = 7;

  Coordinate normal{0, 0, 1};
  Coordinate contactA{0, 0, 0};
  Coordinate contactB{0, 0, 0.1};
  Coordinate comA{0, 0, 0};
  Coordinate comB{0, 0, 0};

  ContactConstraint constraint{
      bodyAIndex, bodyBIndex, normal, contactA, contactB, 0.1, comA, comB, 0.5, 0.0};

  EXPECT_EQ(bodyAIndex, constraint.getBodyAIndex());
  EXPECT_EQ(bodyBIndex, constraint.getBodyBIndex());
}
