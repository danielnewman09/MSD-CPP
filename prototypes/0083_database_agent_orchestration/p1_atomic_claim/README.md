# P1: Atomic Claim Transaction Semantics

## Question

Does `UPDATE ... WHERE status='pending' AND claimed_by IS NULL RETURNING *` inside
a Python `sqlite3` transaction correctly serialize concurrent claims from two threads
on a WAL-mode SQLite file?

## Risk Addressed

**R5** — Concurrent claims may produce duplicate assignments or errors on a shared
SQLite WAL-mode database accessed from multiple Python threads.

## Success Criteria

1. 100 concurrent claim attempts from 2 threads produce exactly N unique claims
   (N = number of available phases — prototype uses N=10)
2. No two threads receive the same `phase_id`
3. No deadlocks or `sqlite3.OperationalError`

## How to Run

No dependencies beyond Python stdlib (Python 3.12+):

```bash
python prototype.py
```

## Expected Output

```
P1: Atomic Claim Transaction Semantics
...
Single Run: PASS
Stress Test (20 trials): ALL TRIALS PASSED
```

## Fallback Strategy

If `BEGIN IMMEDIATE` fails to prevent conflicts, the design fallback is a
`threading.Lock` at the application level. The prototype documents this
in its output.
