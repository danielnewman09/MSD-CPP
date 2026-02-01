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

ECOSData::ECOSData(idxint numVariables, idxint numCones, idxint numEquality)
  : num_variables_{numVariables},
    num_cones_{numCones},
    num_equality_{numEquality},
    G_{},
    A_eq_{},
    h_{},
    c_{},
    b_eq_{},
    cone_sizes_{},
    workspace_{nullptr}
{
  // Reserve storage for vectors to avoid reallocation
  h_.reserve(static_cast<size_t>(numVariables));
  c_.reserve(static_cast<size_t>(numVariables));
  cone_sizes_.reserve(static_cast<size_t>(numCones));
  if (numEquality > 0)
  {
    b_eq_.reserve(static_cast<size_t>(numEquality));
  }
}

ECOSData::ECOSData(ECOSData&& other) noexcept
  : num_variables_{other.num_variables_},
    num_cones_{other.num_cones_},
    num_equality_{other.num_equality_},
    G_{std::move(other.G_)},
    A_eq_{std::move(other.A_eq_)},
    h_{std::move(other.h_)},
    c_{std::move(other.c_)},
    b_eq_{std::move(other.b_eq_)},
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
    num_equality_ = other.num_equality_;
    G_ = std::move(other.G_);
    A_eq_ = std::move(other.A_eq_);
    h_ = std::move(other.h_);
    c_ = std::move(other.c_);
    b_eq_ = std::move(other.b_eq_);
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

  // Validate equality constraint data if present (Ticket 0035b4)
  if (num_equality_ > 0)
  {
    if (A_eq_.nnz == 0)
    {
      throw std::runtime_error(
        "ECOSData::setup: A_eq matrix is empty but num_equality_ = " +
        std::to_string(num_equality_));
    }
    if (static_cast<idxint>(b_eq_.size()) != num_equality_)
    {
      throw std::runtime_error(
        "ECOSData::setup: b_eq size mismatch (expected " +
        std::to_string(num_equality_) + ", got " +
        std::to_string(b_eq_.size()) + ")");
    }
  }

  // Determine equality constraint pointers
  // If num_equality_ > 0, pass A_eq and b_eq; otherwise pass nullptr
  pfloat* Apr = (num_equality_ > 0) ? A_eq_.data.data() : nullptr;
  idxint* Ajc = (num_equality_ > 0) ? A_eq_.col_ptrs.data() : nullptr;
  idxint* Air = (num_equality_ > 0) ? A_eq_.row_indices.data() : nullptr;
  pfloat* beq = (num_equality_ > 0) ? b_eq_.data() : nullptr;

  // Call ECOS_setup() with owned data arrays
  pwork* raw =
    ECOS_setup(num_variables_,  // n (number of variables)
               num_variables_,  // m (number of inequality constraints)
               num_equality_,   // p (number of equality constraints)
               0,  // l (dimension of positive orthant, 0 for SOC-only)
               num_cones_,             // ncones (number of second-order cones)
               cone_sizes_.data(),     // q (array of cone dimensions)
               0,                      // e (exponent cone dimensions, 0)
               G_.data.data(),         // Gpr (CSC sparse matrix data)
               G_.col_ptrs.data(),     // Gjc (CSC column pointers)
               G_.row_indices.data(),  // Gir (CSC row indices)
               Apr,                    // Apr (equality constraint matrix)
               Ajc,                    // Ajc (equality column pointers)
               Air,                    // Air (equality row indices)
               c_.data(),              // c (linear objective)
               h_.data(),              // h (RHS for cone constraints)
               beq                     // b (RHS for equality constraints)
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
