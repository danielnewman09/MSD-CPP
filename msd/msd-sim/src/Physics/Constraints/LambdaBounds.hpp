// Ticket: 0043_constraint_hierarchy_refactor
// Design: docs/designs/0043_constraint_hierarchy_refactor/design.md

#pragma once

#include <limits>

namespace msd_sim {

/**
 * @brief Represents lower and upper bounds on a constraint's Lagrange multiplier
 *
 * LambdaBounds is a value type that encodes the multiplier bound semantics for
 * different constraint types:
 * - Bilateral constraints: λ ∈ (-∞, +∞) — unconstrained multiplier
 * - Unilateral constraints: λ ∈ [0, +∞) — non-negative multiplier (inequalities)
 * - Box-constrained: λ ∈ [lo, hi] — bounded multiplier (friction cone)
 *
 * Factory methods provide semantic construction, and query methods enable
 * solver dispatch without dynamic_cast.
 *
 * @see docs/designs/0043_constraint_hierarchy_refactor/0043_constraint_hierarchy_refactor.puml
 * @ticket 0043_constraint_hierarchy_refactor
 */
struct LambdaBounds {
    double lower;
    double upper;

    /**
     * @brief Create bilateral (equality) constraint bounds: λ ∈ (-∞, +∞)
     * @return LambdaBounds with infinite range in both directions
     */
    [[nodiscard]] static LambdaBounds bilateral() {
        return {-std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()};
    }

    /**
     * @brief Create unilateral (inequality) constraint bounds: λ ∈ [0, +∞)
     * @return LambdaBounds with non-negative range
     */
    [[nodiscard]] static LambdaBounds unilateral() {
        return {0.0, std::numeric_limits<double>::infinity()};
    }

    /**
     * @brief Create box-constrained bounds: λ ∈ [lo, hi]
     * @param lo Lower bound on multiplier
     * @param hi Upper bound on multiplier
     * @return LambdaBounds with custom range
     */
    [[nodiscard]] static LambdaBounds boxConstrained(double lo, double hi) {
        return {lo, hi};
    }

    /**
     * @brief Check if this represents bilateral (equality) constraint bounds
     * @return true if λ ∈ (-∞, +∞)
     */
    [[nodiscard]] bool isBilateral() const {
        return lower == -std::numeric_limits<double>::infinity() &&
               upper ==  std::numeric_limits<double>::infinity();
    }

    /**
     * @brief Check if this represents unilateral (inequality) constraint bounds
     * @return true if λ ∈ [0, +∞)
     */
    [[nodiscard]] bool isUnilateral() const {
        return lower == 0.0 &&
               upper == std::numeric_limits<double>::infinity();
    }

    /**
     * @brief Check if this represents box-constrained bounds
     * @return true if neither bilateral nor unilateral
     */
    [[nodiscard]] bool isBoxConstrained() const {
        return !isBilateral() && !isUnilateral();
    }
};

}  // namespace msd_sim
