// Ticket: 0075a_unified_constraint_data_structure
// Design: docs/designs/0075_unified_contact_constraint/design.md (Phase 1)

#include "msd-sim/src/DataRecorder/DataRecorderVisitor.hpp"
#include "msd-sim/src/DataRecorder/DataRecorder.hpp"

namespace msd_sim {

DataRecorderVisitor::DataRecorderVisitor(DataRecorder& recorder, uint32_t frameId)
  : recorder_{recorder}, frameId_{frameId}
{
}

void DataRecorderVisitor::visit(const msd_transfer::UnifiedContactConstraintRecord& record)
{
  auto recordCopy = record;
  recordCopy.frame.id = frameId_;
  recorder_.getDAO<msd_transfer::UnifiedContactConstraintRecord>().addToBuffer(recordCopy);
}

}  // namespace msd_sim
