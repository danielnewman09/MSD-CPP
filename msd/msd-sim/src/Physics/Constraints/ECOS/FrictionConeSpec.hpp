// Ticket: 0035b_box_constrained_asm_solver
// Design: docs/designs/0035b_box_constrained_asm_solver/design.md

#ifndef MSD_SIM_PHYSICS_ECOS_FRICTION_CONE_SPEC_HPP
#define MSD_SIM_PHYSICS_ECOS_FRICTION_CONE_SPEC_HPP

#include <ecos/ecos.h>
#include <vector>
#include <stdexcept>

namespace msd_sim
{

/**
 * @brief Friction cone specification for ECOS second-order cone constraints
 *
 * For contact-based friction, ECOS needs a second-order cone constraint per contact:
 * - Coulomb friction cone: ||λ_t|| <= μ * λ_n
 * - ECOS cone formulation: ||(λ_t1, λ_t2)|| <= λ_n (after scaling by μ)
 * - Cone dimension: 3 (1 normal + 2 tangential)
 *
 * This struct builds the cone size array required by ECOS from a set of
 * FrictionConstraint instances.
 *
 * Example: 3 contacts with friction coefficients [0.5, 0.8, 0.3]
 * - numContacts = 3
 * - frictionCoefficients = [0.5, 0.8, 0.3]
 * - normalIndices = [0, 3, 6] (constraint indices for normal forces)
 * - getConeSizes() returns [3, 3, 3] (all cones are dimension 3)
 *
 * Constraint ordering convention (per contact i):
 * - Index 3*i: Normal force λ_n
 * - Index 3*i+1: Tangential force λ_t1
 * - Index 3*i+2: Tangential force λ_t2
 *
 * Thread safety: Immutable after construction (thread-safe reads)
 * Error handling: Validates contact count, throws std::invalid_argument on invalid index
 * Memory: Owns friction coefficient and index vectors
 *
 * @see docs/designs/0035b_box_constrained_asm_solver/design.md
 * @ticket 0035b_box_constrained_asm_solver
 */
struct FrictionConeSpec
{
  int numContacts{0};                           // Number of contacts
  std::vector<double> frictionCoefficients;   // μ per contact
  std::vector<int> normalIndices;             // Index of normal constraint per contact

  /**
   * @brief Default constructor creates empty specification
   */
  FrictionConeSpec() = default;

  /**
   * @brief Construct friction cone specification for given number of contacts
   *
   * Reserves space for friction coefficients and normal indices.
   *
   * @param numContacts Number of contacts (must be >= 0)
   * @throws std::invalid_argument if numContacts < 0
   */
  explicit FrictionConeSpec(int numContacts);

  /**
   * @brief Set friction parameters for a specific contact
   *
   * @param contactIndex Index of contact in [0, numContacts)
   * @param mu Friction coefficient (must be >= 0)
   * @param normalConstraintIndex Index of normal constraint in full constraint vector
   * @throws std::invalid_argument if contactIndex out of range or mu < 0
   */
  void setFriction(int contactIndex, double mu, int normalConstraintIndex);

  /**
   * @brief Build ECOS cone size array (all cones are dimension 3 for friction)
   *
   * Returns array of cone sizes for ECOS:
   * - For C contacts: [3, 3, ..., 3] (C entries)
   * - Each friction cone has dimension 3 (1 normal + 2 tangential)
   *
   * @return Vector of cone sizes (all 3)
   */
  [[nodiscard]] std::vector<idxint> getConeSizes() const;

  /**
   * @brief Get number of contacts
   * @return Number of contacts in specification
   */
  [[nodiscard]] int getNumContacts() const { return numContacts; }

  /**
   * @brief Get friction coefficient for specific contact
   * @param contactIndex Index of contact in [0, numContacts)
   * @return Friction coefficient μ for that contact
   * @throws std::invalid_argument if contactIndex out of range
   */
  [[nodiscard]] double getFrictionCoefficient(int contactIndex) const;

  // Rule of Zero (compiler-generated copy/move/destructor)
  FrictionConeSpec(const FrictionConeSpec&) = default;
  FrictionConeSpec& operator=(const FrictionConeSpec&) = default;
  FrictionConeSpec(FrictionConeSpec&&) noexcept = default;
  FrictionConeSpec& operator=(FrictionConeSpec&&) noexcept = default;
  ~FrictionConeSpec() = default;
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_ECOS_FRICTION_CONE_SPEC_HPP
