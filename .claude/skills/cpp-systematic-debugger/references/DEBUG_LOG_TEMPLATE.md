# Debug Log Template

Use this template to track debugging sessions. The debug session is initialized by copying DEBUG_TICKET.md and appending investigation sections.

---

## Investigation Log

### [Timestamp] - Initial Analysis

**Reviewed:**
- [ ] Header file (.h/.hpp) declarations
- [ ] Implementation file (.cpp) 
- [ ] Existing test coverage
- [ ] Build configuration (CMakeLists.txt)

**Observations:**
- [Key observation 1]
- [Key observation 2]

**Memory/Safety Analysis:**
- [ ] Ran with AddressSanitizer
- [ ] Ran with UndefinedBehaviorSanitizer  
- [ ] Ran with ThreadSanitizer (if multithreaded)
- [ ] Ran with Valgrind

**Sanitizer output:**
```
[paste relevant output]
```

---

## Hypotheses Tracker

### H1: [Hypothesis Title]

**Status:** ⏳ Investigating | ✅ Eliminated | ❌ Confirmed | 🔄 Needs info

**Description:** [What might be causing the issue]

**Evidence For:**
- [Evidence point 1]

**Evidence Against:**
- [Evidence point 1]

**Tests to Verify:**
- [ ] Test description

**Conclusion:** [Pending / Eliminated / Confirmed]

---

### H2: [Next Hypothesis]

[Same structure as H1]

---

## Test Results

### Test: [test_name]

**File:** `tests/path/to/test.cpp`  
**Hypothesis:** H1  
**Date Run:** YYYY-MM-DD
**Framework:** GTest / Catch2 / Other

**Build command:**
```bash
cmake --build build --target test_name
```

**Result:** ✅ Pass | ❌ Fail | ⚠️ Error | 💥 Crash

**Output:**
```
[relevant output including any sanitizer messages]
```

**What This Tells Us:**
[Interpretation of results]

---

## Checkpoints

### Checkpoint 1 - [Date]

**Summary:** [Brief summary of progress]

**Human Feedback:** [What the user said/decided]

**Next Actions:** [What was agreed upon]

---

## Resolution

### Root Cause
[Final determination of what caused the issue]

### Category
- [ ] Memory error (leak, use-after-free, buffer overflow)
- [ ] Undefined behavior
- [ ] Logic error
- [ ] Race condition / thread safety
- [ ] API misuse
- [ ] Build/configuration issue
- [ ] Other: _____

### Fix Applied
[Description of the fix, with file/line references]

```cpp
// Before:
[problematic code]

// After:
[fixed code]
```

### Verification
- [ ] Bug no longer reproduces
- [ ] New tests pass
- [ ] Existing tests still pass
- [ ] Sanitizers report clean
- [ ] Valgrind reports clean

### Regression Test Added
**File:** `tests/regression/test_issue_XXX.cpp`
**Covers:** [What edge case this prevents in the future]

---

## Session Notes

[Any additional context, learnings, or observations]