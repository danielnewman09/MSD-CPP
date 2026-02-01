// Ticket: 0035a_tangent_basis_and_friction_constraint
// Design: docs/designs/0035a_tangent_basis_and_friction_constraint/design.md

#ifndef MSD_SIM_PHYSICS_TANGENT_BASIS_HPP
#define MSD_SIM_PHYSICS_TANGENT_BASIS_HPP

#include "msd-sim/src/Environment/Coordinate.hpp"
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace msd_sim
{

/**
 * @brief Orthonormal tangent frame {t1, t2} for a contact normal
 *
 * Represents the two orthonormal tangent directions perpendicular to a contact normal.
 * Used by FrictionConstraint to define friction constraint directions.
 *
 * Invariants (enforced by TangentBasis::computeTangentBasis):
 * - ||t1|| = 1, ||t2|| = 1 (unit length)
 * - t1 · t2 = 0 (orthogonal)
 * - t1 · n = 0, t2 · n = 0 (perpendicular to normal)
 *
 * Thread safety: Value type, safe to copy across threads
 * Error handling: Constructor validates unit length (throws std::invalid_argument if violated)
 *
 * @see docs/designs/0035a_tangent_basis_and_friction_constraint/0035a_tangent_basis_and_friction_constraint.puml
 * @ticket 0035a_tangent_basis_and_friction_constraint
 */
struct TangentFrame {
  Coordinate t1;  // First tangent direction (unit length)
  Coordinate t2;  // Second tangent direction (unit length)

  /**
   * @brief Construct tangent frame from two tangent vectors
   * @param tangent1 First tangent direction (must be unit length)
   * @param tangent2 Second tangent direction (must be unit length)
   * @throws std::invalid_argument if either tangent is not unit length (within 1e-6)
   */
  TangentFrame(const Coordinate& tangent1, const Coordinate& tangent2)
    : t1{tangent1}, t2{tangent2}
  {
    constexpr double kUnitLengthTolerance = 1e-6;
    const double norm1 = t1.norm();
    const double norm2 = t2.norm();

    if (std::abs(norm1 - 1.0) > kUnitLengthTolerance) {
      throw std::invalid_argument(
        "TangentFrame: tangent1 is not unit length (norm = " + std::to_string(norm1) + ")");
    }
    if (std::abs(norm2 - 1.0) > kUnitLengthTolerance) {
      throw std::invalid_argument(
        "TangentFrame: tangent2 is not unit length (norm = " + std::to_string(norm2) + ")");
    }
  }
};

/**
 * @brief Utility namespace for tangent basis construction
 *
 * Implements Duff et al. (2017) algorithm for deterministic, continuous orthonormal
 * basis construction from a single vector.
 *
 * Reference: Duff et al. (2017), "Building an Orthonormal Basis, Revisited", JCGT
 * Mathematical formulation: docs/designs/0035_friction_constraints/M1-tangent-basis.md
 *
 * @ticket 0035a_tangent_basis_and_friction_constraint
 */
namespace TangentBasis {

/**
 * @brief Compute orthonormal tangent basis from contact normal
 *
 * Uses Duff et al. (2017) method: selects the coordinate axis most orthogonal to n,
 * computes t1 via cross product, then t2 = n × t1.
 *
 * Properties:
 * - Orthonormal: ||t1|| = ||t2|| = 1, t1·t2 = 0, ti·n = 0
 * - Deterministic: same n always produces same {t1, t2}
 * - Continuous: small change in n produces small change in tangents
 * - Robust: handles coordinate-aligned normals (no singularities)
 *
 * Algorithm:
 * 1. Select coordinate axis ei with smallest |ni| (most orthogonal to n)
 * 2. Compute t1 = (ei × n) / ||ei × n||
 * 3. Compute t2 = n × t1
 *
 * @param normal Contact normal (must be unit length)
 * @return TangentFrame containing {t1, t2}
 * @throws std::invalid_argument if normal is not unit length (within 1e-6)
 *
 * Thread safety: Stateless function, safe to call from multiple threads
 * Complexity: O(1) - constant time (9 floating-point ops)
 */
inline TangentFrame computeTangentBasis(const Coordinate& normal)
{
  // Validate input: normal must be unit length
  constexpr double kUnitLengthTolerance = 1e-6;
  const double norm = normal.norm();
  if (std::abs(norm - 1.0) > kUnitLengthTolerance) {
    throw std::invalid_argument(
      "TangentBasis::computeTangentBasis: normal is not unit length (norm = " +
      std::to_string(norm) + ")");
  }

  // Extract normal components
  const double nx = normal.x();
  const double ny = normal.y();
  const double nz = normal.z();

  // Select coordinate axis most orthogonal to normal (smallest |ni|)
  // Per Duff et al. (2017), compute t1 = (ei × n) / ||ei × n||
  const double ax = std::abs(nx);
  const double ay = std::abs(ny);
  const double az = std::abs(nz);

  Coordinate t1{};

  if (ax <= ay && ax <= az) {
    // |nx| is smallest (or tied) → select ex = [1, 0, 0]
    // ex × n = [1, 0, 0] × [nx, ny, nz] = [0, -nz, ny]
    const double denom = std::sqrt(ny * ny + nz * nz);
    t1 = Coordinate{0.0, -nz / denom, ny / denom};
  }
  else if (ay <= az) {
    // |ny| is smallest → select ey = [0, 1, 0]
    // ey × n = [0, 1, 0] × [nx, ny, nz] = [-nz, 0, nx]
    const double denom = std::sqrt(nx * nx + nz * nz);
    t1 = Coordinate{-nz / denom, 0.0, nx / denom};
  }
  else {
    // |nz| is smallest → select ez = [0, 0, 1]
    // ez × n = [0, 0, 1] × [nx, ny, nz] = [ny, -nx, 0]
    const double denom = std::sqrt(nx * nx + ny * ny);
    t1 = Coordinate{ny / denom, -nx / denom, 0.0};
  }

  // Compute t2 = n × t1 (automatically unit length and orthogonal to both n and t1)
  Coordinate t2 = normal.cross(t1);

  return TangentFrame{t1, t2};
}

}  // namespace TangentBasis

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_TANGENT_BASIS_HPP
