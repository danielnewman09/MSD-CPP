#include "msd-sim/src/DataRecorder/DataRecorderVisitor.hpp"
#include "msd-sim/src/DataRecorder/DataRecorder.hpp"

namespace msd_sim {

DataRecorderVisitor::DataRecorderVisitor(DataRecorder& recorder, uint32_t frameId)
  : recorder_{recorder}, frameId_{frameId}
{
}

void DataRecorderVisitor::visit(const msd_transfer::ContactConstraintRecord& record)
{
  auto recordCopy = record;
  recordCopy.frame.id = frameId_;
  recorder_.getDAO<msd_transfer::ContactConstraintRecord>().addToBuffer(recordCopy);
}

void DataRecorderVisitor::visit(const msd_transfer::FrictionConstraintRecord& record)
{
  auto recordCopy = record;
  recordCopy.frame.id = frameId_;
  recorder_.getDAO<msd_transfer::FrictionConstraintRecord>().addToBuffer(recordCopy);
}

}  // namespace msd_sim
