---
name: cpp-tutorial-generator
description: Generate companion tutorial documentation with well-documented C++ executables demonstrating mathematical/algorithmic concepts. Use when asked to create tutorials, educational documentation, or explanatory materials that accompany code tasks. Supports agent continuation by reading TUTORIAL_STATE.md artifacts from previous sessions. Outputs to docs/tutorials/ with markdown documentation, algorithm references, and Reveal.js interactive presentations.
---

# C++ Tutorial Generator

Generate educational tutorial documentation alongside production code. Creates beginner-friendly explanations with simple, transparent C++ implementations that reference existing (potentially optimized) codebase objects.

## Workflow Overview

```
1. CHECK CONTINUATION → Look for TUTORIAL_STATE.md from previous agent
2. ANALYZE TASK       → Identify concepts requiring tutorial documentation
3. REFERENCE CODEBASE → Map production objects to tutorial explanations
4. GENERATE DOCS      → Create markdown + C++ examples + Reveal.js presentation
5. SAVE STATE         → Write TUTORIAL_STATE.md for future continuation
```

## Step 1: Check for Continuation State

Before starting, look for `TUTORIAL_STATE.md` in the project root or `docs/tutorials/`:

```bash
find . -name "TUTORIAL_STATE.md" -type f 2>/dev/null | head -5
```

If found, read it to continue previous work. The state file contains:
- Completed tutorial sections
- Pending topics
- Codebase object mappings
- Current presentation slide count

If no state file exists, start fresh and create one at the end.

## Step 2: Analyze the Task

For each coding task that warrants tutorial documentation:

### 2.1 Identify Core Concepts
- Mathematical foundations (algorithms, formulas, theorems)
- Data structures and their properties
- Computational patterns (iteration, recursion, optimization)

### 2.2 Map Complexity Levels
- **Production code**: May use SIMD, templates, caching, or other optimizations
- **Tutorial code**: Simple, readable, single-purpose implementations

### 2.3 Document Object References
Create a mapping table in markdown:

```markdown
| Production Object | Tutorial Equivalent | Concept Explained |
|-------------------|---------------------|-------------------|
| `FastFFT<T>`      | `simple_dft()`      | Discrete Fourier Transform basics |
| `SparseMatrix`    | `dense_matrix`      | Matrix operations before optimization |
```

## Step 3: Reference Existing Codebase

Scan for production objects that the tutorial will reference:

```bash
# Find headers in the codebase
find . -name "*.h" -o -name "*.hpp" | head -20

# Search for class definitions
grep -rn "class\s\+\w\+" --include="*.h" --include="*.hpp" . | head -20
```

Document each referenced object with:
- Its location in the codebase
- What optimization techniques it uses
- Why the tutorial uses a simpler approach

## Step 4: Generate Tutorial Documentation

Output structure in `docs/tutorials/`:

```
docs/tutorials/
├── TUTORIAL_STATE.md          # Continuation state
├── index.md                   # Tutorial index/overview
├── [topic]/
│   ├── README.md              # Main tutorial content
│   ├── algorithm.md           # Algorithm deep-dive with math
│   ├── example.cpp            # Well-documented C++ executable
│   ├── CMakeLists.txt         # Build configuration
│   └── presentation.html      # Reveal.js interactive slides
```

### 4.1 Markdown Documentation (README.md)

Follow this structure:

```markdown
# [Topic Title]

## Overview
[2-3 sentence introduction to the concept]

## Prerequisites
- [Mathematical background needed]
- [C++ knowledge assumed]

## The Problem
[Clear problem statement the code solves]

## Mathematical Foundation
[Equations, theorems, references to external resources]

## Algorithm Walkthrough
[Step-by-step explanation with pseudocode]

## Implementation Notes
[Why the tutorial version differs from production code]

## References
- [Links to papers, textbooks, online resources]

## See Also
- `src/[ProductionClass].h` - Optimized production implementation
```

### 4.2 C++ Tutorial Code (example.cpp)

See `references/cpp-style-guide.md` for complete documentation standards.

Key requirements:
- Every function has a documentation header explaining its purpose
- Mathematical operations include formula comments
- Loop invariants and complexity annotations
- No micro-optimizations—clarity over performance
- Compiles standalone with minimal dependencies
- Includes a `main()` with example usage

### 4.3 Reveal.js Presentation (presentation.html)

See `assets/reveal-template.html` for the base template.

Presentation structure:
1. Title slide with topic and learning objectives
2. Problem motivation (why this matters)
3. Mathematical background (step-by-step derivation)
4. Algorithm visualization (diagrams, animations)
5. Code walkthrough (syntax-highlighted excerpts)
6. Complexity analysis
7. Connection to production code
8. Further reading

Use Reveal.js features:
- Fragments for step-by-step reveals
- Code highlighting with line-by-line focus
- MathJax/KaTeX for equation rendering
- Speaker notes for additional context

## Step 5: Save Continuation State

Always write `docs/tutorials/TUTORIAL_STATE.md` at the end:

```markdown
# Tutorial Generation State

## Session Info
- Last updated: [ISO timestamp]
- Agent session: [identifier if available]

## Completed Topics
- [x] [Topic 1] - docs/tutorials/topic1/
- [x] [Topic 2] - docs/tutorials/topic2/

## Pending Topics
- [ ] [Topic 3] - Needs: [what's missing]
- [ ] [Topic 4] - Blocked on: [dependency]

## Codebase Mappings
| Production | Tutorial | Status |
|------------|----------|--------|
| `ClassA`   | `simple_a.cpp` | Complete |
| `ClassB`   | `simple_b.cpp` | In progress |

## Presentation Progress
- Total slides created: [N]
- Topics with presentations: [list]

## Notes for Next Session
[Any context the next agent should know]
```

## Quick Reference

### Directory Creation
```bash
mkdir -p docs/tutorials/[topic]
```

### Verify Tutorial Structure
```bash
# Check required files exist
ls docs/tutorials/[topic]/{README.md,example.cpp,CMakeLists.txt,presentation.html}
```

### Build Tutorial Example
```bash
cd docs/tutorials/[topic]
mkdir -p build && cd build
cmake ..
make
./example
```

### Serve Reveal.js Locally
```bash
cd docs/tutorials/[topic]
python3 -m http.server 8000
# Open http://localhost:8000/presentation.html
```

## Integration with Production Code

When referencing production objects:
- Use relative paths from the tutorial location
- Include the production header in comments (not compiled)
- Explain what the production version does differently

Example comment pattern:
```cpp
// Production equivalent: src/math/FastFFT.h
// The production version uses:
// - SIMD intrinsics for vectorization
// - Cache-oblivious algorithms
// - Template metaprogramming for compile-time optimization
// This tutorial version prioritizes readability.
```