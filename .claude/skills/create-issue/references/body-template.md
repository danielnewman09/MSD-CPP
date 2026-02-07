# Issue Body Template

Template for converting ticket markdown into a GitHub Issue body.

## Work Ticket Template

```markdown
> **Ticket**: `{ticket_id}` | **Type**: {type} | **Created**: {created}
> **Parent**: #{parent_issue_number} | **Dependencies**: {dependencies} | **Blocks**: {blocks}

---

## Workflow Phases

- [ ] #{design_issue} Design
- [ ] #{math_issue} Math Formulation
- [ ] #{prototype_issue} Prototype
- [ ] #{impl_issue} Implementation

## Overview

{overview_content}

## Requirements

{requirements_content}

## Implementation Approach

{implementation_approach_content}

## Test Plan

{test_plan_content}

## Acceptance Criteria

- [ ] AC1: First criterion
- [x] AC2: Completed criterion

## Files

{files_content}

<details><summary>Workflow Log</summary>

{workflow_log_content}

</details>

---

_Generated from `tickets/{filename}`_
```

## Investigation Ticket Template

```markdown
> **Ticket**: `{ticket_id}` | **Type**: Investigation | **Created**: {created}
> **Parent**: #{parent_issue_number} | **Related**: {related_tickets}

---

## Workflow Phases

- [ ] #{investigate_issue} Investigate
- [ ] #{diagnose_issue} Diagnose
- [ ] #{fix_issue} Fix
- [ ] #{verify_issue} Verify

## Problem Statement

{problem_statement_content}

## Hypothesis Space

{hypothesis_space_content}

## Investigation Plan

{investigation_plan_content}

## Suspect Code

{suspect_code_content}

## Acceptance Criteria

- [ ] AC1: Root cause identified
- [ ] AC2: Fix implemented

<details><summary>Workflow Log</summary>

{workflow_log_content}

</details>

---

_Generated from `tickets/{filename}`_
```

## Feature Ticket (Epic) Template

```markdown
> **Ticket**: `{ticket_id}` | **Type**: {type} | **Created**: {created}
> **Predecessor**: {predecessor}

---

## Overview

{overview_content}

## Subtickets

- [ ] #{issue_number} - Child Title
- [ ] Child Title _(ticket: NNNN, no issue yet)_

## Acceptance Criteria

- [ ] AC1: ...

<details><summary>Background</summary>

{background_content}

</details>

<details><summary>Workflow Log</summary>

{workflow_log_content}

</details>

---

_Generated from `tickets/{filename}`_
```

## Template Rules

### Metadata Banner

The first line is a blockquote banner with key metadata:

```markdown
> **Ticket**: `{ticket_id}` | **Type**: {type} | **Created**: {created}
```

Add a second banner line only if the ticket has parent, dependencies, or blocks:

```markdown
> **Parent**: #{parent_issue} | **Dependencies**: {deps} | **Blocks**: {blocks}
```

- **Parent**: Use `#{number}` if parent has a GitHub issue, otherwise `{ticket_id} _(no issue yet)_`
- **Dependencies**: Comma-separated list of `#{number}` or `ticket_id _(no issue)_`
- **Blocks**: Same format as dependencies

Omit fields that don't apply. If the entire second line would be empty, omit it.

### Workflow Phases Section (Work and Investigation Tickets)

This section appears near the top of Work and Investigation tickets (after the banner, before Overview/Problem Statement). It provides a high-level progress view of the ticket's workflow.

**Work tickets:**
```markdown
## Workflow Phases

- [ ] #42 Design
- [ ] #43 Math Formulation
- [ ] #44 Prototype
- [ ] #45 Implementation
```

**Investigation tickets:**
```markdown
## Workflow Phases

- [ ] #42 Investigate
- [ ] #43 Diagnose
- [ ] #44 Fix
- [ ] #45 Verify
```

Rules:
- Omit the Math Formulation line if math phase doesn't apply (Work tickets only)
- Omit the Prototype line if prototype phase doesn't apply (Work tickets only) — determined by `**Prototype**: No` in ticket metadata
- This section is initially populated with placeholders, then updated after phase sub-issues are created
- Phase sub-issues that already exist (from a previous run) are included with their existing issue numbers

### Sections

Include a section only if it has content. Omit the header entirely for empty sections.

Section mapping from ticket to issue:

**Work and Feature tickets:**

| Ticket Section | Issue Section | Notes |
|---------------|---------------|-------|
| `## Overview` or `## Summary` or `## Motivation` | `## Overview` | Use whichever is present |
| `## Requirements` | `## Requirements` | Include subsections |
| `## Implementation Approach` | `## Implementation Approach` | |
| `## Test Plan` | `## Test Plan` | |
| `## Acceptance Criteria` | `## Acceptance Criteria` | Convert to task list |
| `## Subtickets` | `## Subtickets` | Feature tickets only |
| `## Background` | Wrapped in `<details>` | Feature tickets — collapsible |
| `## Files to Create/Modify` or `## Files` | `## Files` | |
| `## Workflow Log` | Wrapped in `<details>` | Collapsible |

**Investigation tickets** — additional/different section mappings:

| Ticket Section | Issue Section | Notes |
|---------------|---------------|-------|
| `## Problem Statement` | `## Problem Statement` | Primary description for debug tickets |
| `## Hypothesis Space` | `## Hypothesis Space` | Catalog of potential root causes |
| `## Investigation Plan` | `## Investigation Plan` | Phased approach to diagnosis |
| `## Suspect Code` or `## Files to Investigate` | `## Suspect Code` | Code areas to examine |
| `## Diagnostic Tests to Implement` | `## Diagnostic Tests` | Proposed tests |
| `## Systematic Elimination Protocol` | Wrapped in `<details>` | Collapsible — detailed procedure |
| `## Risk Assessment` | Wrapped in `<details>` | Collapsible |

### Acceptance Criteria Formatting

Convert ticket format to GitHub task list:

**Input** (ticket):
```markdown
1. [ ] **AC1**: EnergyTracker class implemented
2. [x] **AC2**: EnergyRecord transfer object created
```

**Output** (issue):
```markdown
- [ ] **AC1**: EnergyTracker class implemented
- [x] **AC2**: EnergyRecord transfer object created
```

Rules:
- Replace numbered list (`1. `) with dash list (`- `)
- Preserve checkbox state (`[ ]` or `[x]`)
- Keep bold AC labels

### Subticket Task List (Feature Tickets Only)

For Feature tickets, build a checkbox list of child work tickets:

```markdown
## Subtickets

- [ ] #12 - Per-Contact Penetration Depth
- [x] #13 - Split Impulse Position Correction _(complete)_
- [ ] Edge Contact Manifold _(ticket: 0040c, no issue yet)_
```

Rules:
- Children with GitHub issues: `- [ ] #{number} - {title}`
- Children without issues: `- [ ] {title} _(ticket: {id}, no issue yet)_`
- Completed children: use `- [x]` checkbox
- Include dependency info if present in the subticket table

### Workflow Log (Collapsible)

Wrap the workflow log in a `<details>` element:

```markdown
<details><summary>Workflow Log</summary>

### Draft Phase
- **Created**: 2026-02-05
- ...

### Design Phase
- ...

</details>
```

### Footer

Always end with:

```markdown
---

_Generated from `tickets/{filename}`_
```

### Sections to Exclude

Do NOT include in the issue body:
- `## Status` — represented by workflow phase sub-issues
- `## Metadata` (old format) — represented by banner and labels
- `## Human Feedback` — internal process section
- `## Deferred Requirements` — include only if non-empty, as a collapsible section
- `## Known Limitation` — include as a regular section if present

### Deferred Requirements (Optional)

If present and non-empty, include as collapsible:

```markdown
<details><summary>Deferred Requirements</summary>

{content}

</details>
```

### Body Length Limit

GitHub limits issue bodies to ~65535 characters. If the rendered body exceeds 60000 characters:

1. Truncate the Workflow Log to the most recent 2 phases
2. Add a note: `_(Workflow log truncated. See full history in tickets/{filename})_`
3. If still too long, truncate Requirements to first 3 subsections with similar note
