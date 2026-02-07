---
name: create-issue
description: Convert a ticket markdown file from tickets/ into a GitHub Issue, preserving all content and metadata. Accepts bare number (0039a), full name (0039a_energy_tracking_diagnostic_infrastructure), or with .md extension. Creates labels, resolves cross-references, updates parent issues, and closes completed tickets. For work tickets, creates workflow phase sub-issues (Design, Math, Prototype, Implementation). For investigation/debug tickets, creates debug phase sub-issues (Investigate, Diagnose, Fix, Verify).
---

# Create GitHub Issue from Ticket

Convert a local markdown ticket into a GitHub Issue with full content preservation, label management, cross-reference resolution, and workflow phase sub-issues.

## Invocation

The user provides:
- **Ticket identifier**: bare number (`0039a`), full filename stem (`0039a_energy_tracking_diagnostic_infrastructure`), or with `.md` extension

Example: `/create-issue 0040b`

## Workflow Overview

```
1.  RESOLVE     -> Find the ticket file in tickets/
2.  PARSE       -> Extract title, metadata, and body sections
3.  CLASSIFY    -> Determine ticket kind: Feature (epic) vs Work (leaf)
4.  CHECK       -> Search for existing issue by ticket label (idempotency)
5.  LABELS      -> Create type, ticket, phase labels if missing
6.  BUILD       -> Format issue body from template
7.  CROSSREF    -> Resolve internal ticket links to issue numbers
8.  CREATE      -> Create or update the GitHub Issue
9.  PHASES      -> (Work tickets only) Create phase sub-issues and link them
10. PARENT      -> Update parent issue's subticket task list
11. REPORT      -> Show URL, labels, phases, and summary
```

## Step 1: Resolve Ticket File

Find the ticket file in `tickets/`:

1. If the argument contains `.md`, look for `tickets/{argument}` directly
2. Otherwise, glob for `tickets/{argument}*.md`
3. If multiple matches, prefer exact prefix match (e.g., `0040` matches `0040_` not `0040a_`)
4. If still ambiguous, list matches and ask the user to clarify

Read the matched file.

## Step 2: Parse Ticket

The project has two ticket header formats. Detect and parse both:

### New Format (Ticket NNNN)

Header line: `# Ticket NNNN: Title Text`

Metadata appears as bold key-value pairs below the Status section:
```
**Current Phase**: value
**Created**: value
**Type**: value
**Prototype**: Yes | No
**Parent Ticket**: [NNNN_name](filename.md)
**Dependencies**: [NNNN](filename.md)
**Blocks**: [NNNN](filename.md)
```

### Old Format (Feature Ticket)

Header line: `# Feature Ticket: Title Text`

Metadata appears in a `## Metadata` section:
```
## Metadata
- **Created**: value
- **Priority**: value
- **Estimated Complexity**: value
- **Target Component(s)**: value
```

### Fields to Extract

| Field | Source (New) | Source (Old) | Required |
|-------|-------------|-------------|----------|
| `ticket_id` | From header `NNNN` | From filename prefix | Yes |
| `title` | After `: ` in `# Ticket NNNN:` | After `: ` in `# Feature Ticket:` | Yes |
| `current_phase` | `**Current Phase**:` | Derive from Status checkboxes | Yes |
| `created` | `**Created**:` | `**Created**:` in Metadata | No |
| `type` | `**Type**:` | None (use `enhancement`) | No |
| `prototype` | `**Prototype**:` (`Yes`/`No`) | None (default `Yes`) | No |
| `parent` | `**Parent Ticket**:` link | None | No |
| `dependencies` | `**Dependencies**:` / `**Depends On**:` links | `## Related Tickets` | No |
| `blocks` | `**Blocks**:` links | None | No |

### Section Extraction

Extract content from these sections (if present):
- `## Overview` or `## Summary` or `## Motivation`
- `## Requirements`
- `## Implementation Approach`
- `## Test Plan`
- `## Acceptance Criteria`
- `## Subtickets`
- `## Files to Create/Modify` or `## Files`
- `## Workflow Log`

### Status Derivation

If `**Current Phase**:` is not present, derive from the Status checkbox list — the last checked item indicates the current phase.

## Step 3: Classify Ticket

Determine the ticket kind: **Feature** (epic/parent), **Work** (leaf), or **Investigation** (debug).

### Feature Ticket (Epic)

A ticket is a Feature if ANY of these are true:
- `Type` field contains `(Parent)` — e.g., `Implementation (Parent)`, `Debug Investigation (Parent)`
- The ticket has a `## Subtickets` section with a table of child tickets
- The ticket body is primarily a tracking document for child work items

Feature tickets:
- Get child work tickets as sub-issues (from the Subtickets table)
- Do NOT get workflow phase sub-issues
- Get the `feature` label

### Investigation Ticket (Debug)

A ticket is an Investigation if ANY of these are true:
- `Type` field contains `Investigation` (but NOT `(Parent)`) — e.g., `Investigation`, `Debug Investigation`
- The ticket has a `## Hypothesis Space` or `## Problem Statement` section
- The ticket has a `## Suspect Code` or `## Files to Investigate` section

Investigation tickets:
- Get debug workflow phase sub-issues (Investigate, Diagnose, Fix, Verify)
- May be a sub-issue of a parent Feature ticket
- Get the `type:investigation` label

### Work Ticket (Leaf)

All other tickets are Work tickets. These represent actual units of work that go through the design-to-implementation workflow.

Work tickets:
- Get workflow phase sub-issues (Design, Math, Prototype, Implementation)
- May be a sub-issue of a parent Feature ticket

## Step 4: Check for Existing Issue

Search for an existing issue with the ticket label:

```bash
gh issue list --label "ticket:{ticket_id}" --state all --json number,title,state --limit 1
```

- If found and `--state open`: offer to **update** the existing issue
- If found and `--state closed`: report that the issue already exists and is closed; ask if user wants to reopen and update
- If not found: proceed to create

Also check for existing phase sub-issues (for Work and Investigation tickets):
```bash
# Work ticket phases
gh issue list --label "ticket:{ticket_id}" --label "phase:design" --state all --json number --limit 1
gh issue list --label "ticket:{ticket_id}" --label "phase:math" --state all --json number --limit 1
gh issue list --label "ticket:{ticket_id}" --label "phase:prototype" --state all --json number --limit 1
gh issue list --label "ticket:{ticket_id}" --label "phase:implementation" --state all --json number --limit 1

# Investigation ticket phases
gh issue list --label "ticket:{ticket_id}" --label "phase:investigate" --state all --json number --limit 1
gh issue list --label "ticket:{ticket_id}" --label "phase:diagnose" --state all --json number --limit 1
gh issue list --label "ticket:{ticket_id}" --label "phase:fix" --state all --json number --limit 1
gh issue list --label "ticket:{ticket_id}" --label "phase:verify" --state all --json number --limit 1
```

Only check the phases relevant to the ticket kind. Skip creating any phase sub-issue that already exists.

## Step 5: Create Labels

Check and create labels as needed using `gh label create`. See `references/label-mapping.md` for the full mapping.

Required labels for every issue:
1. **Ticket label**: `ticket:{ticket_id}` (color `#0E8A16`) — idempotency key
2. **Type label**: derived from the `Type` metadata field

For Feature tickets:
3. **Feature label**: `feature` (color `#FBCA04`)

For Work tickets, create phase labels:
3. `phase:design` (color `#C2E0C6`)
4. `phase:math` (color `#F9D0C4`) — only if math phase applies
5. `phase:prototype` (color `#FEF2C0`) — only if prototype phase applies
6. `phase:implementation` (color `#BFD4F2`)

For Investigation tickets, create debug phase labels:
3. `phase:investigate` (color `#D4C5F9`)
4. `phase:diagnose` (color `#F9D0C4`)
5. `phase:fix` (color `#C2E0C6`)
6. `phase:verify` (color `#BFD4F2`)

Create each label only if it doesn't already exist:
```bash
gh label create "ticket:{ticket_id}" --color "0E8A16" --description "Ticket {ticket_id}" --force
gh label create "type:{type}" --color "{color}" --force
gh label create "phase:design" --color "C2E0C6" --description "Design phase" --force
```

Use `--force` to avoid errors if the label already exists.

## Step 6: Build Issue Body

Use the template from `references/body-template.md`.

Rules:
- **Omit empty sections entirely** — do not include section headers with no content
- **Acceptance Criteria**: Convert `- [ ] **AC1**: text` or `- [x] text` to GitHub task list format `- [ ] text` / `- [x] text`
- **Subtickets** (Feature tickets only): Build a task list of children
- **Workflow Phases** (Work and Investigation tickets): Include a task list of phase sub-issues (populated after Step 9)
- **Workflow Log**: Wrap in `<details>` collapsible
- **Max body length**: GitHub limits issue body to ~65535 chars. If body exceeds 60000 chars, truncate the Workflow Log section and add a note

### Subticket Task List (Feature Tickets)

For Feature tickets with a `## Subtickets` section:

1. Parse the subticket table for ticket IDs and titles
2. For each subticket, check if a GitHub issue exists: `gh issue list --label "ticket:{child_id}" --state all --json number,title --limit 1`
3. Format:
   - Has issue: `- [ ] #{issue_number} - {title}`
   - No issue yet: `- [ ] {title} _(ticket: {child_id}, no issue yet)_`
   - If the child ticket's phase indicates completion, use `- [x]` instead

### Workflow Phases Section (Work and Investigation Tickets)

For Work and Investigation tickets, include a `## Workflow Phases` section with placeholders. This section is updated after phase sub-issues are created in Step 9.

**Work tickets:**
```markdown
## Workflow Phases

- [ ] #{design_issue} Design
- [ ] #{math_issue} Math Formulation
- [ ] #{prototype_issue} Prototype
- [ ] #{impl_issue} Implementation
```

Omit the Math line if math phase doesn't apply. Omit the Prototype line if prototype phase doesn't apply.

**Investigation tickets:**
```markdown
## Workflow Phases

- [ ] #{investigate_issue} Investigate
- [ ] #{diagnose_issue} Diagnose
- [ ] #{fix_issue} Fix
- [ ] #{verify_issue} Verify
```

## Step 7: Resolve Cross-References

Scan the issue body for internal ticket links in these patterns:
- `[NNNN](NNNN_name.md)` — markdown link to another ticket
- `[NNNN_name](NNNN_name.md)` — full name link
- `ticket NNNN` or `Ticket NNNN` — plain text reference

For each referenced ticket:
1. Check if a GitHub issue exists: `gh issue list --label "ticket:{ref_id}" --state all --json number --limit 1`
2. If found: replace the link with `#N` (the GitHub issue number)
3. If not found: keep the original text but append ` _(no issue yet)_`

Track and report all link resolutions in Step 11.

## Step 8: Create or Update Main Issue

### Create New Issue

```bash
gh issue create \
  --title "{title}" \
  --body "{body}" \
  --label "ticket:{ticket_id}" \
  --label "type:{type}" \
  [--label "feature"]
```

Capture the issue number from the output URL.

### Update Existing Issue

```bash
gh issue edit {issue_number} \
  --title "{title}" \
  --body "{body}"
```

Also ensure all labels are applied:
```bash
gh issue edit {issue_number} --add-label "ticket:{ticket_id},type:{type}"
```

### Close Completed Issues

If the ticket's status indicates completion (phase contains "Complete" or "Merged"), close the issue after creation:

```bash
gh issue close {issue_number} --reason "completed"
```

## Step 9: Create Phase Sub-Issues (Work and Investigation Tickets)

Skip this step entirely for Feature tickets.

### 9a: Determine Applicable Phases

**Work tickets** get these phases:
1. **Design** — always
2. **Math Formulation** — only if math is detected (see detection rules below)
3. **Prototype** — only if prototype is needed (see detection rules below)
4. **Implementation** — always

**Investigation tickets** get these phases:
1. **Investigate** — always (instrument, reproduce, gather evidence)
2. **Diagnose** — always (hypothesis elimination, root cause identification)
3. **Fix** — always (implement the fix)
4. **Verify** — always (regression testing, sanitizer validation)

### 9b: Math Phase Detection (Work Tickets Only)

Include the Math Formulation phase if ANY of these are true:
- The ticket body contains keywords: `physics`, `collision`, `inertia`, `quaternion`, `tensor`, `eigenvalue`, `impulse`, `friction`, `torque`, `angular`, `kinetic energy`, `potential energy`, `Lagrangian`, `Jacobian`, `numerical stability`, `convergence`
- A file exists at `docs/designs/{ticket_name}/math-formulation.md`
- The ticket's `## Requirements` section contains LaTeX-style math (backtick blocks with equations, `½`, `ω`, `∑`, etc.)

Where `{ticket_name}` is the full filename stem without `.md` (e.g., `0040b_split_impulse_position_correction`).

### 9b2: Prototype Phase Detection (Work Tickets Only)

Include the Prototype phase unless the ticket explicitly opts out. Check in this order:

1. If the ticket has a `**Prototype**:` metadata field set to `No` (case-insensitive), **skip** the Prototype phase
2. If the `**Prototype**:` field is `Yes` or absent, **include** the Prototype phase (default behavior)

This allows ticket authors to opt out of prototyping for straightforward work like API refactors, renames, or simple bug fixes where there are no design assumptions to validate.

### 9c: Create Each Phase Sub-Issue

For each applicable phase, create a sub-issue:

```bash
gh issue create \
  --title "[Phase] {title}" \
  --body "{phase_body}" \
  --label "ticket:{ticket_id}" \
  --label "phase:{phase}"
```

**Work ticket** phase titles:
- `[Design] Split Impulse Position Correction`
- `[Math] Split Impulse Position Correction`
- `[Prototype] Split Impulse Position Correction`
- `[Implementation] Split Impulse Position Correction`

**Investigation ticket** phase titles:
- `[Investigate] Collision Energy Stabilization Debug`
- `[Diagnose] Collision Energy Stabilization Debug`
- `[Fix] Collision Energy Stabilization Debug`
- `[Verify] Collision Energy Stabilization Debug`

### Phase Sub-Issue Body Content

See `references/phase-templates.md` for the body content of each phase sub-issue. Work and Investigation tickets use different templates.

### 9d: Link Phase Sub-Issues to Parent

After creating each phase sub-issue, link it as a sub-issue of the main ticket issue using the GitHub GraphQL API:

```bash
# Get the node ID of the main issue
PARENT_ID=$(gh issue view {main_issue_number} --json id -q '.id')

# Get the node ID of the phase sub-issue
CHILD_ID=$(gh issue view {phase_issue_number} --json id -q '.id')

# Link as sub-issue (use -F for GraphQL variables, -f for query)
gh api graphql -F parentId="$PARENT_ID" -F childId="$CHILD_ID" -f query='
mutation AddSubIssue($parentId: ID!, $childId: ID!) {
  addSubIssue(input: {
    issueId: $parentId
    subIssueId: $childId
  }) {
    issue { id }
    subIssue { id }
  }
}'
```

If the GraphQL API fails (e.g., sub-issues not enabled for the repo), fall back to including the phase issues as a task list in the main issue body.

### 9e: Update Main Issue Body with Phase Links

After creating all phase sub-issues, update the main issue's `## Workflow Phases` section to include the actual issue numbers:

**Work tickets:**
```markdown
## Workflow Phases

- [ ] #42 Design
- [ ] #43 Math Formulation
- [ ] #44 Prototype
- [ ] #45 Implementation
```

Omit Math and/or Prototype lines if those phases don't apply.

**Investigation tickets:**
```markdown
## Workflow Phases

- [ ] #42 Investigate
- [ ] #43 Diagnose
- [ ] #44 Fix
- [ ] #45 Verify
```

```bash
gh issue edit {main_issue_number} --body "{updated_body}"
```

## Step 10: Update Parent Issue

If the ticket has a `**Parent Ticket**:` field:

1. Find the parent's GitHub issue: `gh issue list --label "ticket:{parent_id}" --state all --json number,body --limit 1`
2. If the parent issue exists and is a Feature ticket:
   - Find the `## Subtickets` section in the parent body
   - Replace the line for this child ticket with the linked version: `- [ ] #{new_issue} - {title}`
   - Use `gh issue edit {parent_number} --body "{updated_body}"`
3. Link this issue as a sub-issue of the parent (same GraphQL as Step 9d)

If the parent issue doesn't exist yet, skip this step and note it in the report.

## Step 11: Report

Print a summary:

```
Issue created: #{number} — {title}
URL: {url}
Kind: Feature | Work | Investigation
Labels: ticket:{id}, type:{type}, [feature]

Workflow phases created:          (Work and Investigation tickets)
  Work:
    - #42 [Design] {title}
    - #43 [Math] {title}
    - #44 [Prototype] {title}
    - #45 [Implementation] {title}
  Investigation:
    - #42 [Investigate] {title}
    - #43 [Diagnose] {title}
    - #44 [Fix] {title}
    - #45 [Verify] {title}
  Sub-issue linking: success | fallback to task list

Cross-references resolved:
  - ticket 0038 -> #12
  - ticket 0040a -> #15
  - ticket 0040d (no issue yet)

Parent update:
  - Updated #{parent} subticket list with #{number}
  - Linked as sub-issue of #{parent}: success | skipped
```

## Error Handling

### Ticket file not found
- List all files in `tickets/` that start with the given prefix
- If none match, report "No ticket found matching '{input}'"

### `gh` not authenticated
- If any `gh` command fails with auth error, tell the user to run `gh auth login`

### Label creation fails
- Use `--force` flag which updates existing labels rather than failing
- If `gh label create` still fails, report the error and continue without the label

### Sub-issue API not available
- If the GraphQL `addSubIssue` mutation fails, fall back to task list references in the issue body
- Report that sub-issue linking was skipped and why

### Parent issue body too large to parse
- If the parent body can't be reliably modified, skip the parent update and report it

### Phase sub-issue already exists
- If a phase sub-issue with the matching `ticket:{id}` + `phase:{phase}` labels already exists, skip creating it
- Use the existing issue number in the Workflow Phases section

## References

- See `references/label-mapping.md` for label naming conventions and color codes
- See `references/body-template.md` for the issue body template
- See `references/phase-templates.md` for phase sub-issue body templates
