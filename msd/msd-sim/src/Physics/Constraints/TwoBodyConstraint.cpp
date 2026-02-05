// Ticket: 0032_contact_constraint_refactor
// Design: docs/designs/0032_contact_constraint_refactor/design.md

#include "msd-sim/src/Physics/Constraints/TwoBodyConstraint.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <stdexcept>
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

Eigen::VectorXd TwoBodyConstraint::evaluate(
    const InertialState& /* state */,
    double /* time */) const
{
  throw std::logic_error(
      "TwoBodyConstraint::evaluate() - Two-body constraints must use evaluateTwoBody(). "
      "This error indicates API misuse.");
}

Eigen::MatrixXd TwoBodyConstraint::jacobian(
    const InertialState& /* state */,
    double /* time */) const
{
  throw std::logic_error(
      "TwoBodyConstraint::jacobian() - Two-body constraints must use jacobianTwoBody(). "
      "This error indicates API misuse.");
}

bool TwoBodyConstraint::isActive(
    const InertialState& /* state */,
    double /* time */) const
{
  throw std::logic_error(
      "TwoBodyConstraint::isActive() - Two-body constraints must use isActiveTwoBody(). "
      "This error indicates API misuse.");
}

}  // namespace msd_sim
