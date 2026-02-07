# Label Mapping Reference

Label naming conventions and color codes for the `create-issue` skill.

## Type Labels

Derived from the ticket's `**Type**:` metadata field.

| Type Value (in ticket) | GitHub Label | Color | Description |
|------------------------|-------------|-------|-------------|
| Implementation | `type:implementation` | `0075CA` | Feature implementation work |
| Implementation (Parent) | `type:implementation` + `feature` | `0075CA` / `FBCA04` | Feature/epic tracking ticket |
| Infrastructure | `type:infrastructure` | `D4C5F9` | Build, tooling, or plumbing work |
| Test Suite | `type:test-suite` | `BFD4F2` | Test-only ticket |
| Investigation | `type:investigation` | `FEF2C0` | Research or debugging ticket |
| _(no Type field)_ | `enhancement` | _(GitHub default)_ | Fallback for old-format tickets |

## Feature Label

If the ticket is classified as a Feature/Epic (has `(Parent)` in Type or has a `## Subtickets` section), add the `feature` label.

| Label | Color | Description |
|-------|-------|-------------|
| `feature` | `FBCA04` | Feature/epic that tracks child work tickets |

## Phase Labels — Work Tickets

Applied to workflow phase sub-issues of Work tickets. Each phase sub-issue gets the `ticket:{NNNN}` label AND the corresponding phase label.

| Phase | GitHub Label | Color | Description |
|-------|-------------|-------|-------------|
| Design | `phase:design` | `C2E0C6` | Architectural design phase |
| Math Formulation | `phase:math` | `F9D0C4` | Mathematical formulation phase |
| Prototype | `phase:prototype` | `FEF2C0` | Prototype validation phase |
| Implementation | `phase:implementation` | `BFD4F2` | Production implementation phase |

## Phase Labels — Investigation Tickets

Applied to debug workflow phase sub-issues of Investigation tickets. Each phase sub-issue gets the `ticket:{NNNN}` label AND the corresponding phase label.

| Phase | GitHub Label | Color | Description |
|-------|-------------|-------|-------------|
| Investigate | `phase:investigate` | `D4C5F9` | Instrument, reproduce, gather evidence |
| Diagnose | `phase:diagnose` | `F9D0C4` | Hypothesis elimination, root cause ID |
| Fix | `phase:fix` | `C2E0C6` | Implement the fix |
| Verify | `phase:verify` | `BFD4F2` | Regression testing, sanitizer validation |

## Ticket Labels (Idempotency Key)

Every issue gets a unique ticket label used as the idempotency key for duplicate detection. Phase sub-issues share the same ticket label as their parent so they can be queried together.

| Label Format | Color | Example |
|-------------|-------|---------|
| `ticket:{NNNN}` | `0E8A16` | `ticket:0039a`, `ticket:0040` |

The ticket ID is extracted from the filename prefix (everything before the first `_`).

## Label Creation Commands

```bash
# Ticket label (always created, one per ticket)
gh label create "ticket:0039a" --color "0E8A16" --description "Ticket 0039a" --force

# Type labels (create once, reused across tickets)
gh label create "type:implementation" --color "0075CA" --description "Feature implementation" --force
gh label create "type:infrastructure" --color "D4C5F9" --description "Build/tooling/plumbing" --force
gh label create "type:test-suite" --color "BFD4F2" --description "Test-only ticket" --force
gh label create "type:investigation" --color "FEF2C0" --description "Research/debugging" --force

# Feature label (for epics/parents)
gh label create "feature" --color "FBCA04" --description "Feature/epic tracking ticket" --force

# Work ticket phase labels (create once, reused)
gh label create "phase:design" --color "C2E0C6" --description "Design phase" --force
gh label create "phase:math" --color "F9D0C4" --description "Math formulation phase" --force
gh label create "phase:prototype" --color "FEF2C0" --description "Prototype validation phase" --force
gh label create "phase:implementation" --color "BFD4F2" --description "Implementation phase" --force

# Investigation ticket phase labels (create once, reused)
gh label create "phase:investigate" --color "D4C5F9" --description "Investigate phase" --force
gh label create "phase:diagnose" --color "F9D0C4" --description "Diagnose phase" --force
gh label create "phase:fix" --color "C2E0C6" --description "Fix phase" --force
gh label create "phase:verify" --color "BFD4F2" --description "Verify phase" --force
```

The `--force` flag updates existing labels instead of erroring, making these commands idempotent.
