# Tutorial README Template

Use this structure when creating `README.md` files for tutorial topics.

---

# {{TOPIC_TITLE}}

## Overview

[2-3 sentences introducing the concept. What is it? Why does it matter?]

## Prerequisites

**Mathematical Background:**
- [Concept 1 - e.g., "Linear algebra basics (vectors, matrices)"]
- [Concept 2 - e.g., "Complex numbers and exponentials"]

**C++ Knowledge:**
- [Skill 1 - e.g., "STL containers (std::vector)"]
- [Skill 2 - e.g., "Basic templates"]

## The Problem

[Clear problem statement. What challenge does this algorithm/technique solve?]

**Input:** [Description of input data]

**Output:** [Description of expected output]

**Example:**
```
Input:  [concrete example]
Output: [expected result]
```

## Mathematical Foundation

[Core mathematical concepts. Use LaTeX-style notation where helpful.]

### Key Formula

$$
[Main equation]
$$

Where:
- $x$ = [definition]
- $y$ = [definition]

### Derivation (Optional)

[Step-by-step derivation if it aids understanding]

1. Start with [initial assumption]
2. Apply [transformation]
3. Simplify to get [result]

## Algorithm Walkthrough

### Pseudocode

```
function algorithm_name(input):
    // Step 1: Initialize
    ...
    
    // Step 2: Main computation
    for each element:
        ...
    
    // Step 3: Finalize
    return result
```

### Step-by-Step Explanation

**Step 1: Initialization**
[Explain what this step does and why]

**Step 2: Main Loop**
[Explain the core computation]

**Step 3: Finalization**
[Explain post-processing]

### Complexity Analysis

| Metric | Complexity | Explanation |
|--------|------------|-------------|
| Time   | O(...)     | [Why this complexity] |
| Space  | O(...)     | [What memory is used] |

## Implementation Notes

### Tutorial vs Production

This tutorial implementation prioritizes **clarity over performance**. 

| Aspect | Tutorial | Production (`{{PRODUCTION_PATH}}`) |
|--------|----------|-------------------------------------|
| Algorithm | [Simple version] | [Optimized version] |
| Data structures | [Basic] | [Specialized] |
| Optimizations | None | [List optimizations] |

### Why the Difference?

The production code uses [optimization techniques] because:
1. [Reason 1]
2. [Reason 2]

For learning purposes, the simpler approach helps you understand:
1. [Learning goal 1]
2. [Learning goal 2]

## Building and Running

```bash
# From the tutorial directory
mkdir -p build && cd build
cmake ..
make
./example
```

**Expected Output:**
```
[Sample output from running the example]
```

## Interactive Presentation

Open `presentation.html` in a browser for an interactive walkthrough:

```bash
# Option 1: Direct file open
open presentation.html  # macOS
xdg-open presentation.html  # Linux

# Option 2: Local server (better for some features)
python3 -m http.server 8000
# Then open http://localhost:8000/presentation.html
```

**Keyboard shortcuts in presentation:**
- `Space` / `→` : Next slide
- `←` : Previous slide
- `Esc` : Overview mode
- `S` : Speaker notes
- `F` : Fullscreen

## References

### Primary Sources
- [Author, "Title", Publication, Year](URL) - [Brief note on relevance]

### Further Reading
- [Resource 1](URL) - [Description]
- [Resource 2](URL) - [Description]

### Online Tools
- [Tool 1](URL) - [What it's useful for]

## See Also

- **Production implementation:** `{{PRODUCTION_PATH}}`
- **Related tutorials:** 
  - [`../related-topic/`](../related-topic/) - [How it relates]
- **Test coverage:** `{{TEST_PATH}}`

---

*This tutorial is part of the {{PROJECT_NAME}} documentation.*
*Last updated: {{DATE}}*