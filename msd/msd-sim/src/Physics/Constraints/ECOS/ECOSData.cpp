// Ticket: 0035b2_ecos_data_wrapper
// Design: docs/designs/0035b2_ecos_data_wrapper/design.md

#include "msd-sim/src/Physics/Constraints/ECOS/ECOSData.hpp"
#include <stdexcept>

namespace msd_sim
{

void ECOSWorkspaceDeleter::operator()(pwork* w) const noexcept
{
  if (w != nullptr)
  {
    ECOS_cleanup(w, 0);
  }
}

ECOSData::ECOSData(idxint numVariables, idxint numCones)
  : num_variables_{numVariables},
    num_cones_{numCones},
    G_{},
    h_{},
    c_{},
    cone_sizes_{},
    workspace_{nullptr}
{
  // Reserve storage for vectors to avoid reallocation
  h_.reserve(static_cast<size_t>(numVariables));
  c_.reserve(static_cast<size_t>(numVariables));
  cone_sizes_.reserve(static_cast<size_t>(numCones));
}

ECOSData::ECOSData(ECOSData&& other) noexcept
  : num_variables_{other.num_variables_},
    num_cones_{other.num_cones_},
    G_{std::move(other.G_)},
    h_{std::move(other.h_)},
    c_{std::move(other.c_)},
    cone_sizes_{std::move(other.cone_sizes_)},
    workspace_{std::move(other.workspace_)}
{
}

ECOSData& ECOSData::operator=(ECOSData&& other) noexcept
{
  if (this != &other)
  {
    // CRITICAL: Clean up our workspace FIRST, while our data arrays (G_, h_,
    // c_) are still valid. ECOS_cleanup() calls unset_equilibration() which
    // writes to the data arrays pointed to by the workspace.
    cleanup();

    // Now safe to move data arrays
    num_variables_ = other.num_variables_;
    num_cones_ = other.num_cones_;
    G_ = std::move(other.G_);
    h_ = std::move(other.h_);
    c_ = std::move(other.c_);
    cone_sizes_ = std::move(other.cone_sizes_);
    workspace_ = std::move(other.workspace_);
  }
  return *this;
}

void ECOSData::setup()
{
  // Precondition: workspace must not already be set up
  // (Calling setup() twice without cleanup() in between would leak the first
  // workspace)
  if (workspace_ != nullptr)
  {
    throw std::runtime_error(
      "ECOSData::setup: workspace already set up (call cleanup() first)");
  }

  // Precondition validation
  if (G_.nnz == 0)
  {
    throw std::runtime_error("ECOSData::setup: G matrix is empty (nnz == 0)");
  }

  if (static_cast<idxint>(h_.size()) != num_variables_)
  {
    throw std::runtime_error("ECOSData::setup: h size mismatch (expected " +
                             std::to_string(num_variables_) + ", got " +
                             std::to_string(h_.size()) + ")");
  }

  if (static_cast<idxint>(c_.size()) != num_variables_)
  {
    throw std::runtime_error("ECOSData::setup: c size mismatch (expected " +
                             std::to_string(num_variables_) + ", got " +
                             std::to_string(c_.size()) + ")");
  }

  if (static_cast<idxint>(cone_sizes_.size()) != num_cones_)
  {
    throw std::runtime_error(
      "ECOSData::setup: cone_sizes size mismatch (expected " +
      std::to_string(num_cones_) + ", got " +
      std::to_string(cone_sizes_.size()) + ")");
  }

  // Call ECOS_setup() with owned data arrays
  pwork* raw =
    ECOS_setup(num_variables_,  // n (number of variables)
               num_variables_,  // m (number of inequality constraints)
               0,  // p (number of equality constraints, 0 for friction)
               0,  // l (dimension of positive orthant, 0 for SOC-only)
               num_cones_,             // ncones (number of second-order cones)
               cone_sizes_.data(),     // q (array of cone dimensions)
               0,                      // e (exponent cone dimensions, 0)
               G_.data.data(),         // Gpr (CSC sparse matrix data)
               G_.col_ptrs.data(),     // Gjc (CSC column pointers)
               G_.row_indices.data(),  // Gir (CSC row indices)
               nullptr,                // Apr (no equality constraints)
               nullptr,                // Ajc (no equality constraints)
               nullptr,                // Air (no equality constraints)
               c_.data(),              // c (linear objective)
               h_.data(),              // h (RHS for cone constraints)
               nullptr                 // b (RHS for equality constraints, NULL)
    );

  if (raw == nullptr)
  {
    throw std::runtime_error("ECOS_setup failed: invalid problem data");
  }

  // unique_ptr takes ownership, will call ECOSWorkspaceDeleter on destruction
  workspace_.reset(raw);
}

void ECOSData::cleanup()
{
  // Delegate to unique_ptr::reset(), which is idempotent
  // If workspace_ is non-null, ECOSWorkspaceDeleter is invoked
  // If workspace_ is null, this is a no-op
  workspace_.reset();
}

}  // namespace msd_sim
