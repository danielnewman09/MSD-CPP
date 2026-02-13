#pragma once

namespace msd_transfer {

// Forward declarations
struct ContactConstraintRecord;
struct FrictionConstraintRecord;

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
 */
class ConstraintRecordVisitor {
public:
  virtual ~ConstraintRecordVisitor() = default;

  /**
   * @brief Visit a contact constraint record
   * @param record The contact constraint state to process
   */
  virtual void visit(const ContactConstraintRecord& record) = 0;

  /**
   * @brief Visit a friction constraint record
   * @param record The friction constraint state to process
   */
  virtual void visit(const FrictionConstraintRecord& record) = 0;
};

}  // namespace msd_transfer
