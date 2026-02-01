// Ticket: 0035b_box_constrained_asm_solver
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#include "msd-sim/src/Physics/Constraints/ECOS/FrictionConeSpec.hpp"

namespace msd_sim
{

FrictionConeSpec::FrictionConeSpec(int numContacts)
  : numContacts{numContacts}
{
  if (numContacts < 0)
  {
    throw std::invalid_argument(
        "FrictionConeSpec: numContacts must be non-negative (got " +
        std::to_string(numContacts) + ")");
  }

  frictionCoefficients.reserve(static_cast<size_t>(numContacts));
  normalIndices.reserve(static_cast<size_t>(numContacts));
}

void FrictionConeSpec::setFriction(int contactIndex, double mu, int normalConstraintIndex)
{
  if (contactIndex < 0 || contactIndex >= numContacts)
  {
    throw std::invalid_argument(
        "FrictionConeSpec::setFriction: contactIndex out of range [0, " +
        std::to_string(numContacts) + ") (got " + std::to_string(contactIndex) + ")");
  }

  if (mu < 0.0)
  {
    throw std::invalid_argument(
        "FrictionConeSpec::setFriction: friction coefficient must be non-negative (got " +
        std::to_string(mu) + ")");
  }

  // Ensure vectors are sized appropriately
  if (static_cast<int>(frictionCoefficients.size()) <= contactIndex)
  {
    frictionCoefficients.resize(static_cast<size_t>(contactIndex) + 1, 0.0);
  }

  if (static_cast<int>(normalIndices.size()) <= contactIndex)
  {
    normalIndices.resize(static_cast<size_t>(contactIndex) + 1, -1);
  }

  frictionCoefficients[static_cast<size_t>(contactIndex)] = mu;
  normalIndices[static_cast<size_t>(contactIndex)] = normalConstraintIndex;
}

std::vector<idxint> FrictionConeSpec::getConeSizes() const
{
  std::vector<idxint> coneSizes{};
  coneSizes.reserve(static_cast<size_t>(numContacts));

  // Each friction cone has dimension 3 (1 normal + 2 tangential)
  for (int i = 0; i < numContacts; ++i)
  {
    coneSizes.push_back(3);
  }

  return coneSizes;
}

double FrictionConeSpec::getFrictionCoefficient(int contactIndex) const
{
  if (contactIndex < 0 || contactIndex >= numContacts)
  {
    throw std::invalid_argument(
        "FrictionConeSpec::getFrictionCoefficient: contactIndex out of range [0, " +
        std::to_string(numContacts) + ") (got " + std::to_string(contactIndex) + ")");
  }

  if (static_cast<int>(frictionCoefficients.size()) <= contactIndex)
  {
    // Not yet set, return 0.0 as default
    return 0.0;
  }

  return frictionCoefficients[static_cast<size_t>(contactIndex)];
}

}  // namespace msd_sim
