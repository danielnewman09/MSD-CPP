---
name: cpp-systematic-debugger
description: C++ debugging workflow for investigating complex issues. Use when asked to debug C++ code, especially when the user provides a DEBUG_TICKET.md file or points to specific .cpp/.h files, classes, or functions. This skill guides systematic elimination of potential causes using C++ tools (GTest, Catch2, sanitizers, Valgrind), maintains documentation of investigated paths, proposes and implements diagnostic tests, and supports human-in-the-loop feedback throughout the debugging process.
---

# C++ Systematic Debugger

An interactive workflow for methodically debugging C++ issues through hypothesis elimination, test-driven investigation, and documented reasoning.

## Workflow Overview

```
1. PARSE TICKET → Read DEBUG_TICKET.md for issue details
2. INVESTIGATE  → Review code, headers, and existing tests
3. ELIMINATE    → Rule out causes with evidence
4. PROPOSE      → Suggest diagnostic tests
5. IMPLEMENT    → Write and run tests (GTest/Catch2)
6. CHECKPOINT   → Get human feedback, iterate or resolve
```

## Step 0: Debug Ticket

Before beginning, look for `DEBUG_TICKET.md` in the project root or current directory. This file contains the structured problem description from the user.

If no ticket exists, ask the user to create one using the template in `assets/DEBUG_TICKET_TEMPLATE.md`.

**Required ticket sections:**
- Problem description and symptoms
- Suspect files/classes/functions
- Build environment and compiler info
- Reproduction steps

## Step 1: Initialize Debug Session

Create a debug log file to track the investigation:

```bash
mkdir -p .debug-sessions
SESSION_ID=$(date +%Y%m%d_%H%M%S)
cp DEBUG_TICKET.md .debug-sessions/debug_${SESSION_ID}.md
```

Append investigation sections to the log. See `references/debug-log-template.md` for the full format.

## Step 2: Investigate Suspect Code

For each file/class/function identified in the ticket:

### 2.1 Header and Documentation Review
- Read header files (.h/.hpp) for class declarations and API contracts
- Check Doxygen comments and inline documentation
- Look for `@pre`, `@post`, `@throws` annotations
- Note any `static_assert`, `concept`, or SFINAE constraints
- Review `#define` guards and macro usage

### 2.2 Implementation Analysis
- Trace data flow through suspect .cpp files
- Identify memory management patterns (RAII, raw pointers, smart pointers)
- Check for common C++ pitfalls:
  - Use-after-free / dangling references
  - Iterator invalidation
  - Object slicing
  - Undefined behavior (signed overflow, null deref, uninitialized reads)
  - Thread safety issues (data races, deadlocks)
- Review exception safety guarantees
- Check move/copy semantics

### 2.3 Test Coverage Analysis
```bash
# Find existing tests
find . -name "*test*.cpp" -o -name "*_test.cpp" -o -name "test_*.cpp" | head -20

# Check for test framework
grep -r "gtest\|catch\|doctest\|boost.test" --include="CMakeLists.txt" .

# If using gcov/lcov:
# lcov --capture --directory . --output-file coverage.info
# lcov --list coverage.info | grep "suspect_file"
```

**Document in debug log:**
- Class hierarchy and ownership model
- Thread safety assumptions
- Existing test coverage (or gaps)
- Initial observations related to the issue

## Step 3: Systematic Elimination

For each potential cause, document:

```markdown
### Hypothesis: [Brief description]

**Status:** ⏳ Investigating | ✅ Eliminated | ❌ Confirmed as cause | 🔄 Needs more info

**Evidence gathered:**
- [What was checked]
- [What was found]

**Conclusion:** [Why this is/isn't the cause]
```

**Elimination methods:**
1. **Code inspection** - Logic errors, off-by-one, null checks
2. **Test verification** - Does existing test cover this case?
3. **Log analysis** - Add logging, review output
4. **Reproduction** - Minimal test case that triggers issue
5. **Bisection** - Narrow down to specific commit/change

## Step 4: Propose Diagnostic Tests

Based on investigation, propose tests that will:

1. **Isolate the issue** - Test suspect component in isolation
2. **Verify assumptions** - Test preconditions and postconditions
3. **Cover edge cases** - Test boundaries, nulls, empty inputs
4. **Reproduce the bug** - Test that currently fails, will pass when fixed

**Test proposal format:**
```markdown
### Proposed Test: [Name]

**Purpose:** What this test will prove or disprove
**Target:** Which component/function/behavior
**Type:** Unit | Integration | Regression
**Expected outcome:** What we expect to learn

**Test outline:**
- Setup: [preconditions]
- Action: [what to test]
- Assert: [expected results]
```

Present proposals to user before implementing.

## Step 5: Implement Diagnostic Tests

After user approval, implement tests using the project's test framework.

### 5.1 Test Framework Detection
```bash
# Check CMakeLists.txt for test framework
grep -E "gtest|GTest|catch|Catch2|doctest|Boost.*Test" CMakeLists.txt

# Common locations
ls tests/ test/ unittest/ 2>/dev/null
```

### 5.2 GTest Implementation Pattern
```cpp
// tests/debug_investigation_test.cpp
#include <gtest/gtest.h>
#include "suspect_class.h"

// Link to hypothesis H1 in debug log
TEST(DebugInvestigation, H1_BoundaryCondition) {
    SuspectClass obj;
    // Test the specific condition from hypothesis
    EXPECT_EQ(obj.method(edge_case_input), expected_output);
}

TEST(DebugInvestigation, H2_NullHandling) {
    SuspectClass obj;
    EXPECT_NO_THROW(obj.method(nullptr));
}
```

### 5.3 Catch2 Implementation Pattern
```cpp
// tests/debug_investigation_test.cpp
#include <catch2/catch_test_macros.hpp>
#include "suspect_class.h"

TEST_CASE("Debug H1: Boundary condition", "[debug][h1]") {
    SuspectClass obj;
    REQUIRE(obj.method(edge_case_input) == expected_output);
}

TEST_CASE("Debug H2: Null handling", "[debug][h2]") {
    SuspectClass obj;
    REQUIRE_NOTHROW(obj.method(nullptr));
}
```

### 5.4 Build and Run Tests
```bash
# CMake build
mkdir -p build && cd build
cmake .. -DBUILD_TESTING=ON
cmake --build . --target debug_investigation_test

# Run specific test
./debug_investigation_test --gtest_filter="DebugInvestigation.*"
# or for Catch2:
./debug_investigation_test "[debug]"
```

### 5.5 Run with Sanitizers (if not already enabled)
```bash
# Rebuild with sanitizers for deeper investigation
cmake .. -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -g"
cmake --build .
./debug_investigation_test
```

### 5.6 Document Results
Update debug log with:
- Test code location
- Build configuration used
- Execution results (including sanitizer output)
- What was learned
- Updated hypothesis status

## Step 6: Human Checkpoint

**At each checkpoint, present:**

```markdown
## Debug Session Checkpoint

### Progress Summary
- Hypotheses eliminated: [count]
- Hypotheses confirmed: [count]  
- Hypotheses pending: [count]

### Key Findings
[Most important discoveries]

### Current Best Theory
[What we think is happening and why]

### Recommended Next Steps
1. [Option A - describe and estimate effort]
2. [Option B - describe and estimate effort]

### Questions for You
- [Specific questions that would help narrow investigation]
```

**Wait for human feedback before:**
- Implementing new tests
- Pursuing a new hypothesis track
- Making any code changes
- Marking the investigation complete

## Iteration Patterns

**If tests reveal the cause:**
1. Document the root cause clearly
2. Propose fix with test that verifies it
3. Suggest additional regression tests

**If tests are inconclusive:**
1. Identify what information is still missing
2. Propose more targeted tests
3. Consider expanding investigation scope

**If human redirects investigation:**
1. Document why previous path was deprioritized
2. Update hypotheses with new information
3. Continue with new direction

## Quick Reference Commands

```bash
# View current debug session
cat .debug-sessions/debug_*.md | tail -100

# Search for patterns in codebase
grep -rn "pattern" --include="*.cpp" --include="*.h" --include="*.hpp" .

# Find recent changes to suspect files
git log --oneline -20 -- path/to/file.cpp

# Show class/function usages
grep -rn "ClassName\|function_name" --include="*.cpp" --include="*.h" .

# Find all includes of a header
grep -rn '#include.*"suspect.h"' --include="*.cpp" .

# Build with debug symbols
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Run with Valgrind (memory errors)
valgrind --leak-check=full --track-origins=yes ./executable

# Run with AddressSanitizer output
ASAN_OPTIONS=detect_leaks=1:print_stats=1 ./executable

# Generate coverage report
lcov --capture --directory . --output-file coverage.info
genhtml coverage.info --output-directory coverage_report

# Check for undefined behavior
./executable  # if built with -fsanitize=undefined

# Examine core dump
gdb ./executable core
```

## C++ Debugging Checklist

Common C++ issues to check:
- [ ] Memory leaks (Valgrind, ASan)
- [ ] Use-after-free (ASan)
- [ ] Buffer overflows (ASan)
- [ ] Uninitialized memory (MSan, Valgrind)
- [ ] Data races (TSan)
- [ ] Undefined behavior (UBSan)
- [ ] Integer overflow (UBSan)
- [ ] Null pointer dereference
- [ ] Iterator invalidation
- [ ] Exception safety violations
- [ ] RAII/resource management issues
- [ ] ODR violations (different definitions in TUs)

## Integration with User Workflow

- Always explain reasoning before taking actions
- Prefer asking clarifying questions over making assumptions
- Keep debug log updated as single source of truth
- Respect user's time - batch questions when possible
- Celebrate progress and acknowledge user insights