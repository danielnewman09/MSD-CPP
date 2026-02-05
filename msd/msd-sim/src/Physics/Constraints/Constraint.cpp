// Ticket: 0031_generalized_lagrange_constraints
// Design: docs/designs/0031_generalized_lagrange_constraints/design.md

#include "msd-sim/src/Physics/Constraints/Constraint.hpp"
#include <Eigen/src/Core/Matrix.h>
#include "msd-sim/src/Physics/RigidBody/InertialState.hpp"

namespace msd_sim
{

Eigen::VectorXd Constraint::partialTimeDerivative(const InertialState& /* state */,
                                                  double /* time */) const
{
  // Default implementation: zero vector (no explicit time dependence)
  Eigen::VectorXd result = Eigen::VectorXd::Zero(dimension());
  return result;
}

}  // namespace msd_sim
