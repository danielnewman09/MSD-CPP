# Iteration Log — {feature-name}

> **Purpose**: Track every build-test cycle during implementation or investigation. Agents MUST consult this log before each new change to avoid repeating failed approaches.
>
> **Location**: `docs/designs/{feature-name}/iteration-log.md` (feature tickets) or `docs/investigations/{feature-name}/iteration-log.md` (investigation tickets)
>
> **Circle Detection**: Before making changes, check for:
> - Same file modified 3+ times with similar changes
> - Test results oscillating between iterations (A fixed / B broken, then B fixed / A broken)
> - Same hypothesis attempted with the same approach
>
> If a circle is detected: STOP, document the pattern below, and escalate to the human.

**Ticket**: 0058_constraint_ownership_cleanup
**Branch**: 0058-constraint-ownership-cleanup
**Baseline**: 713/717 tests passing (4 known failures: H3, B1, B2, B5)

---

## Circle Detection Flags

_None detected._

---

## Iterations

### Iteration 1 — {YYYY-MM-DD HH:MM}
**Commit**: {short SHA}
**Hypothesis**: {Why this change was made — what problem it's solving}
**Changes**:
- `path/to/file.cpp`: {description of change}
**Build Result**: PASS / FAIL ({details if fail})
**Test Result**: {pass}/{total} — {list of new failures or fixes vs previous iteration}
**Impact vs Previous**: {+N passes, -N regressions, net change}
**Assessment**: {Does this move us forward? Any unexpected side effects?}
