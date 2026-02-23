#pragma once

// Ticket: 0075a_unified_constraint_data_structure
// Design: docs/designs/0075_unified_contact_constraint/design.md (Phase 1)

namespace msd_transfer {

// Forward declarations
struct UnifiedContactConstraintRecord;

/**
 * @brief Visitor interface for type-safe constraint record dispatching
 *
 * This interface enables polymorphic constraint recording without dynamic_cast
 * or std::any. Each concrete constraint type calls the appropriate visit()
 * overload, and the compiler enforces that all constraint types are handled.
 *
 * To add a new constraint type:
 * 1. Add new visit(const NewConstraintRecord&) overload to this interface
 * 2. Implement the overload in all concrete visitors (e.g., DataRecorderVisitor)
 * 3. The constraint calls visitor.visit(record) in its recordState() method
 *
 * Compiler will catch missing visit() implementations at build time.
 *
 * @ticket 0075a_unified_constraint_data_structure
 */
class ConstraintRecordVisitor {
public:
  virtual ~ConstraintRecordVisitor() = default;

  /**
   * @brief Visit a unified contact constraint record
   *
   * Handles both frictionless (tangent fields zeroed) and frictional contacts.
   *
   * @param record The unified contact constraint state to process
   */
  virtual void visit(const UnifiedContactConstraintRecord& record) = 0;
};

}  // namespace msd_transfer
