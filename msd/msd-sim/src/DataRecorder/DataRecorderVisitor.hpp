#pragma once

// Ticket: 0075a_unified_constraint_data_structure
// Design: docs/designs/0075_unified_contact_constraint/design.md (Phase 1)

#include "msd-transfer/src/ConstraintRecordVisitor.hpp"
#include "msd-transfer/src/UnifiedContactConstraintRecord.hpp"

namespace msd_sim {

// Forward declaration
class DataRecorder;

/**
 * @brief Concrete visitor that buffers constraint records to DataRecorder DAOs
 *
 * Implements the ConstraintRecordVisitor interface to enable type-safe constraint
 * state recording without dynamic_cast or std::any_cast. Each visit() method
 * receives a typed constraint record, sets the frame FK, and buffers it to the
 * appropriate DAO.
 *
 * As of ticket 0075a, the unified ContactConstraint dispatches a single
 * UnifiedContactConstraintRecord instead of separate ContactConstraintRecord
 * and FrictionConstraintRecord records.
 *
 * Usage:
 * ```cpp
 * DataRecorderVisitor visitor{recorder, frameId};
 * constraint->recordState(visitor, bodyAId, bodyBId);  // Dispatches to visit()
 * ```
 *
 * Thread safety: Not thread-safe (caller must ensure DataRecorder thread safety)
 * Error handling: Exceptions from DAO operations propagate to caller
 *
 * @see msd_transfer::ConstraintRecordVisitor
 * @see DataRecorder
 * @ticket 0075a_unified_constraint_data_structure
 */
class DataRecorderVisitor : public msd_transfer::ConstraintRecordVisitor {
public:
  /**
   * @brief Construct visitor wrapping DataRecorder and frame ID
   *
   * @param recorder DataRecorder instance to buffer records to
   * @param frameId Frame ID for FK references
   */
  DataRecorderVisitor(DataRecorder& recorder, uint32_t frameId);

  /**
   * @brief Visit unified contact constraint record
   *
   * Sets frame FK and buffers record to UnifiedContactConstraintRecord DAO.
   * Handles both frictionless contacts (tangent fields zeroed) and frictional
   * contacts (full tangent basis and impulse components).
   *
   * @param record Unified contact constraint state to buffer
   */
  void visit(const msd_transfer::UnifiedContactConstraintRecord& record) override;

private:
  DataRecorder& recorder_;
  uint32_t frameId_;
};

}  // namespace msd_sim
