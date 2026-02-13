#pragma once

#include "msd-transfer/src/ConstraintRecordVisitor.hpp"
#include "msd-transfer/src/ContactConstraintRecord.hpp"
#include "msd-transfer/src/FrictionConstraintRecord.hpp"

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
 * Usage:
 * ```cpp
 * DataRecorderVisitor visitor{recorder, frameId};
 * constraint->recordState(visitor, bodyAId, bodyBId);  // Dispatches to appropriate visit()
 * ```
 *
 * Thread safety: Not thread-safe (caller must ensure DataRecorder thread safety)
 * Error handling: Exceptions from DAO operations propagate to caller
 *
 * @see msd_transfer::ConstraintRecordVisitor
 * @see DataRecorder
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
   * @brief Visit contact constraint record
   *
   * Sets frame FK and buffers record to ContactConstraintRecord DAO.
   *
   * @param record Contact constraint state to buffer
   */
  void visit(const msd_transfer::ContactConstraintRecord& record) override;

  /**
   * @brief Visit friction constraint record
   *
   * Sets frame FK and buffers record to FrictionConstraintRecord DAO.
   *
   * @param record Friction constraint state to buffer
   */
  void visit(const msd_transfer::FrictionConstraintRecord& record) override;

private:
  DataRecorder& recorder_;
  uint32_t frameId_;
};

}  // namespace msd_sim
