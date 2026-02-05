// Ticket: 0035b2_ecos_data_wrapper
// Design: docs/designs/0035b2_ecos_data_wrapper/design.md

#ifndef MSD_SIM_PHYSICS_CONSTRAINTS_ECOS_ECOS_DATA_HPP
#define MSD_SIM_PHYSICS_CONSTRAINTS_ECOS_ECOS_DATA_HPP

#include <ecos/ecos.h>
#include "msd-sim/src/Physics/Constraints/ECOS/ECOSSparseMatrix.hpp"
#include <memory>
#include <vector>

namespace msd_sim
{

/**
 * @brief Custom deleter for ECOS workspace pointer
 *
 * Calls ECOS_cleanup() when std::unique_ptr destroys the workspace.
 * The deleter is noexcept because ECOS_cleanup() is a C function that does not throw.
 *
 * Note: std::unique_ptr guarantees it will not invoke the deleter with nullptr,
 * so we do not need a null check here. However, we include it defensively for safety.
 *
 * @see ECOSData
 * @ticket 0035b2_ecos_data_wrapper
 */
struct ECOSWorkspaceDeleter
{
  void operator()(pwork* w) const noexcept;
};

/// Type alias for managed ECOS workspace pointer with automatic cleanup
using ECOSWorkspacePtr = std::unique_ptr<pwork, ECOSWorkspaceDeleter>;

/**
 * @brief RAII wrapper for ECOS solver workspace and problem data
 *
 * ECOSData manages the lifecycle of an ECOS solver workspace, ensuring that
 * ECOS_cleanup() is called automatically when the object is destroyed. It owns
 * all data arrays passed to ECOS (G matrix, h vector, c vector, cone sizes),
 * guaranteeing their validity for the lifetime of the workspace.
 *
 * Usage pattern:
 * 1. Construct ECOSData with problem dimensions
 * 2. Populate G, h, c, cone_sizes
 * 3. Call setup() to create ECOS workspace
 * 4. Use workspace.get() or workspace-> to access ECOS workspace for solving
 * 5. Cleanup is automatic via unique_ptr destructor
 *
 * Thread safety: Not thread-safe. Each thread must have its own ECOSData instance.
 *
 * Memory management: Move-only (unique_ptr is non-copyable). Workspace cleanup
 * via ECOSWorkspaceDeleter is automatic and exception-safe.
 *
 * @see docs/designs/0035b2_ecos_data_wrapper/design.md
 * @ticket 0035b2_ecos_data_wrapper
 */
struct ECOSData
{
  // Problem dimensions
  idxint num_variables_{0};       // Decision variables (3C)
  idxint num_cones_{0};           // Number of second-order cones (C)
  idxint num_equality_{0};        // Number of equality constraints (Ticket 0035b4)

  // Owned sparse matrix storage (CSC format for G)
  ECOSSparseMatrix G_;

  // Owned sparse matrix storage for equality constraints (CSC format for A_eq)
  // Ticket: 0035b4_ecos_solve_integration
  ECOSSparseMatrix A_eq_;

  // Owned constraint vectors
  std::vector<pfloat> h_;      // RHS for inequality constraints
  std::vector<pfloat> c_;      // Linear objective
  std::vector<pfloat> b_eq_;   // RHS for equality constraints (Ticket 0035b4)

  // Owned cone dimensions
  std::vector<idxint> cone_sizes_;

  // ECOS workspace (owned via unique_ptr with custom deleter)
  // IMPORTANT: Declared last so it is destroyed first. ECOS_cleanup() (called
  // by ECOSWorkspaceDeleter) accesses G_, h_, and c_ data arrays during
  // unset_equilibration(), so those members must still be alive when the
  // workspace is destroyed.
  ECOSWorkspacePtr workspace_{nullptr};

  /**
   * @brief Construct ECOSData with problem dimensions
   *
   * Initializes dimensions and reserves storage for vectors. Does NOT call
   * ECOS_setup() — use setup() after populating G, h, c, cone_sizes.
   *
   * @param numVariables Number of decision variables (typically 3*numContacts)
   * @param numCones Number of second-order cones (typically numContacts)
   * @param numEquality Number of equality constraints (default 0)
   */
  explicit ECOSData(idxint numVariables, idxint numCones, idxint numEquality = 0);

  /**
   * @brief Destructor (default)
   *
   * Compiler-generated destructor destroys workspace_ unique_ptr, which
   * automatically calls ECOSWorkspaceDeleter if workspace is non-null.
   * No manual cleanup needed — this is RAII via unique_ptr.
   */
  ~ECOSData() = default;

  /**
   * @brief Move constructor
   *
   * Transfers ownership of workspace and data arrays. Source workspace is
   * nullified (no double-free possible). Data arrays are moved before
   * workspace to maintain pointer validity.
   */
  ECOSData(ECOSData&& other) noexcept;

  /**
   * @brief Move assignment
   *
   * Cleans up current workspace FIRST (while our data arrays are still valid),
   * then transfers ownership from source. This ordering is critical because
   * ECOS_cleanup() accesses the data arrays (G, h, c) during
   * unset_equilibration().
   */
  ECOSData& operator=(ECOSData&& other) noexcept;

  /**
   * @brief Copy construction deleted
   *
   * ECOS workspace cannot be deep-copied (unique_ptr is non-copyable).
   */
  ECOSData(const ECOSData&) = delete;

  /**
   * @brief Copy assignment deleted
   *
   * ECOS workspace cannot be deep-copied (unique_ptr is non-copyable).
   */
  ECOSData& operator=(const ECOSData&) = delete;

  /**
   * @brief Setup ECOS workspace by calling ECOS_setup()
   *
   * Call this method after populating G, h, c, and cone_sizes with problem data.
   * If equality constraints are needed, also populate A_eq_ and b_eq_ before calling.
   * Creates the ECOS workspace and stores it in workspace_ unique_ptr.
   *
   * Preconditions:
   * - workspace_ must be null (not already set up). Call cleanup() first if reusing.
   * - G.nnz > 0 (G matrix has been populated)
   * - h.size() == num_variables_
   * - c.size() == num_variables_
   * - cone_sizes.size() == num_cones_
   * - If num_equality_ > 0: A_eq_.nnz > 0 and b_eq_.size() == num_equality_
   *
   * Postconditions:
   * - workspace_ != nullptr (if successful)
   * - ECOS workspace ready for ECOS_solve() calls
   *
   * @throws std::runtime_error if preconditions violated or ECOS_setup() fails
   *
   * Exception safety: Strong guarantee. If ECOS_setup() fails or preconditions
   * are violated, workspace_ remains unchanged (null).
   *
   * @ticket 0035b4_ecos_solve_integration (equality constraint support)
   */
  void setup();

  /**
   * @brief Check if ECOS workspace is active
   *
   * @return true if setup() has been called successfully, false otherwise
   */
  [[nodiscard]] bool isSetup() const
  {
    return workspace_ != nullptr;
  }

  /**
   * @brief Release ECOS workspace (triggers ECOS_cleanup via deleter)
   *
   * Resets workspace_ to null. If workspace was non-null, ECOSWorkspaceDeleter
   * calls ECOS_cleanup(). Safe to call multiple times (idempotent).
   *
   * Exception safety: No-throw guarantee (reset() invokes noexcept deleter)
   */
  void cleanup();
};

}  // namespace msd_sim

#endif  // MSD_SIM_PHYSICS_CONSTRAINTS_ECOS_ECOS_DATA_HPP
