---
name: create-ticket
description: Generate a fully-formatted ticket file from a natural language feature description, then create the corresponding GitHub Issue with phase sub-issues via /create-issue.
---

# Create Ticket from Natural Language Prompt

Generate a structured ticket markdown file from a free-form feature description, write it to `tickets/`, and invoke `/create-issue` to push it to GitHub.

## Invocation

The user provides:
- **Feature description**: A free-form text prompt describing what they want built

Example: `/create-ticket "Add energy tracking to the simulation engine"`
Example: `/create-ticket Build a Python replay dashboard with Three.js frontend`

The argument is everything after `/create-ticket`. Quotes are optional.

## Workflow Overview

```
1. PARSE PROMPT  -> Extract feature intent, components, complexity signals
2. ASSIGN NUMBER -> Find highest ticket number in tickets/, increment by 1
3. GENERATE NAME -> Convert feature title to snake_case filename
4. FILL TEMPLATE -> Read ticket template, populate fields
5. WRITE TICKET  -> Write to tickets/{NNNN}_{name}.md
6. CREATE ISSUE  -> Invoke /create-issue skill with the new ticket identifier
7. REPORT        -> Show ticket path, issue URL, and summary
```

## Step 1: Parse Prompt

Analyze the user's free-form description to extract:

1. **Feature title**: A concise title (3-8 words, imperative form). E.g., "Add Energy Tracking to Simulation Engine"
2. **Summary**: A one-paragraph summary of the feature
3. **Motivation**: Why this feature is needed
4. **Functional requirements**: Numbered list of concrete requirements (extract from the prompt, infer reasonable ones)
5. **Non-functional requirements**: Performance, memory, thread safety constraints (if mentioned)
6. **Acceptance criteria**: Testable criteria derived from the requirements
7. **Metadata signals**: See `references/prompt-analysis.md` for keyword-based detection of:
   - Target components
   - Languages
   - Whether math design is required
   - Estimated complexity
   - Priority

If the prompt is too vague to generate meaningful requirements (fewer than 2 words, or no discernible feature intent), ask the user for more detail before proceeding.

## Step 2: Assign Ticket Number

Scan `tickets/` for the highest numeric prefix:

1. List all files in `tickets/` matching pattern `[0-9]*_*.md`
2. Extract the numeric prefix from each filename (the part before the first non-numeric character, e.g., `0072c` -> `0072`, `0074` -> `0074`)
3. Find the maximum base number
4. The new ticket number is `max + 1`, zero-padded to 4 digits

**Important**: Only consider base numbers (strip letter suffixes like `a`, `b`, `c`). For example, if the highest files are `0072c_...` and `0074_...`, the next number is `0075`.

## Step 3: Generate Filename

Convert the feature title to a snake_case filename:

1. Take the feature title from Step 1
2. Convert to lowercase
3. Replace spaces and hyphens with underscores
4. Remove special characters (keep only `[a-z0-9_]`)
5. Collapse multiple underscores to single
6. Trim leading/trailing underscores
7. Filename: `{NNNN}_{snake_case_name}.md`

Example: "Add Energy Tracking to Simulation Engine" -> `0075_add_energy_tracking_to_simulation_engine.md`

## Step 4: Fill Template

Read the template from `.claude/templates/ticket.md.template`.

Populate the template fields:

### Header
- Replace `{FEATURE_NAME}` with the extracted feature title

### Status
- Mark `[x] Draft` (first checkbox)
- All other checkboxes remain `[ ]`
- **Remove conditional status lines** that don't apply:
  - If `Requires Math Design: No`, remove all lines containing `(if Requires Math Design: Yes)`
  - If Languages is only `C++`, remove lines containing `(if multi-language)`, `(if Python in Languages)`, `(if Frontend in Languages)`
  - If `Generate Tutorial: No`, remove lines containing `(if Generate Tutorial: Yes)`
  - If Languages includes `Python`, keep `(if Python in Languages)` lines but strip the condition text
  - If Languages includes `Frontend`, keep `(if Frontend in Languages)` lines but strip the condition text
  - If Languages has 2+ entries, keep `(if multi-language)` lines but strip the condition text
- After removing/keeping conditional lines, strip the condition annotations (the `(if ...)` text) from remaining lines

### Metadata
- **Created**: Today's date in `YYYY-MM-DD` format
- **Author**: Run `git config user.name` to get the author name. If unavailable, use "AI-generated"
- **Priority**: Detected from prompt (see `references/prompt-analysis.md`), default `Medium`
- **Estimated Complexity**: Detected from prompt, default `Medium`
- **Target Component(s)**: Detected components, or `TBD` if unclear
- **Languages**: Detected languages, default `C++`
- **Generate Tutorial**: Default `No` unless the prompt explicitly mentions tutorial/educational content
- **Requires Math Design**: Detected from math keywords (see `references/prompt-analysis.md`), default `No`
- **GitHub Issue**: Leave empty (will be filled by `/create-issue`)

### Content Sections
- **Summary**: Synthesized from the prompt — one clear paragraph
- **Motivation**: Synthesized from the prompt — why this matters
- **Functional Requirements**: Numbered list extracted/inferred from the prompt
- **Non-Functional Requirements**: Fill in from prompt signals, or use sensible defaults with `{TBD}` markers
- **Constraints**: Extract from prompt or leave with `{TBD}` markers
- **Acceptance Criteria**: Generate testable criteria matching the functional requirements

### Sections to Leave as Templates
- **Design Decisions (Human Input)**: Keep the template placeholders — these are for human input
- **References**: Populate `Related Code` if specific files/classes are mentioned in the prompt, otherwise keep placeholders
- **Human Feedback**: Keep the template placeholders

### Workflow Log
- **Remove conditional workflow log sections** that don't apply:
  - If `Requires Math Design: No`, remove "Math Design Phase" and "Math Design Review Phase" sections
  - If Languages is only `C++`, remove "Integration Design Phase", "Integration Design Review Phase", "Python Design Phase", "Frontend Design Phase" sections
  - If `Generate Tutorial: No`, remove "Tutorial Documentation Phase" section
  - Keep condition annotations stripped from remaining section headers
- If Languages includes Python, keep the Python Design Phase section
- If Languages includes Frontend, keep the Frontend Design Phase section
- If multi-language, keep Integration Design sections

### Human Feedback Section
- **Remove conditional feedback subsections** that don't apply:
  - If `Requires Math Design: No`, remove "Feedback on Math Design" subsection
  - If Languages is only `C++`, remove "Feedback on Integration Design", "Feedback on Python Design", "Feedback on Frontend Design"
  - If `Generate Tutorial: No`, remove "Feedback on Tutorial" subsection

## Step 5: Write Ticket

Write the populated template to `tickets/{NNNN}_{name}.md`.

After writing, display to the user:
```
Ticket created: tickets/{NNNN}_{name}.md
```

## Step 6: Create GitHub Issue

Invoke the `/create-issue` skill with the new ticket number:

```
/create-issue {NNNN}
```

This will:
- Create the GitHub Issue with proper labels
- Create phase sub-issues (Design, Math, Prototype, Implementation as applicable)
- Link sub-issues to the parent
- Report the issue URL

## Step 7: Report

After `/create-issue` completes, provide a combined summary:

```
Ticket created: tickets/{filename}
  Title: {title}
  Components: {components}
  Languages: {languages}
  Math Design: {yes/no}
  Complexity: {complexity}

GitHub Issue: #{number} — {url}
  Phase sub-issues: {list}
```

## Error Handling

### Prompt too vague
- If the description is fewer than 2 meaningful words, ask the user to provide more detail
- Example: `/create-ticket "fix"` -> "Could you describe the feature in more detail? What should be fixed and where?"

### Ticket number conflict
- After determining the next number, verify no file with that prefix exists
- If it does (race condition), increment by 1 and retry

### Template not found
- If `.claude/templates/ticket.md.template` doesn't exist, report the error and suggest the user check their repository

### `/create-issue` failure
- If the GitHub issue creation fails, the ticket file still exists locally
- Report the local ticket path and suggest the user run `/create-issue {NNNN}` manually after fixing the issue

## References

- See `references/prompt-analysis.md` for keyword detection rules
- See `.claude/templates/ticket.md.template` for the ticket template
- See `.claude/skills/create-issue/SKILL.md` for the GitHub issue creation workflow
