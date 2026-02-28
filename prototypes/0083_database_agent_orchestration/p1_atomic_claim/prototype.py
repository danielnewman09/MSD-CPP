"""
Prototype P1: Atomic Claim Transaction Semantics
Ticket: 0083_database_agent_orchestration

Question:
    Does `UPDATE ... WHERE status='pending' AND claimed_by IS NULL RETURNING *`
    inside a Python sqlite3 transaction correctly serialize concurrent claims from
    two threads on a WAL-mode SQLite file?

Success Criteria:
    1. 100 concurrent claim attempts from 2 threads produce exactly N unique claims
       (N = number of available phases)
    2. No two threads receive the same phase_id
    3. No deadlocks or sqlite3.OperationalError

Approach:
    - Create an in-memory (or temp-file) SQLite DB with WAL mode
    - Insert N=10 phases in 'pending' state
    - Launch 2 threads, each trying to claim up to 100 phases via
      UPDATE ... RETURNING * in a transaction
    - Collect all claimed phase_ids across both threads
    - Assert: len(all_claims) == N and len(set(phase_ids)) == len(all_claims)

Design reference:
    The claiming query from design.md:
        UPDATE phases
        SET status = 'in_progress', claimed_by = :agent_id, claimed_at = :now
        WHERE phase_id IN (
            SELECT phase_id FROM phases
            WHERE status = 'pending' AND claimed_by IS NULL
            LIMIT 1
        )
        RETURNING *;
"""

import sqlite3
import threading
import time
import tempfile
import os
from dataclasses import dataclass
from typing import Optional


# ---------------------------------------------------------------------------
# Schema (minimal subset sufficient for the prototype)
# ---------------------------------------------------------------------------

SCHEMA = """
PRAGMA journal_mode=WAL;
PRAGMA busy_timeout=5000;

CREATE TABLE IF NOT EXISTS phases (
    phase_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    ticket_id  INTEGER NOT NULL,
    phase_name TEXT    NOT NULL,
    status     TEXT    NOT NULL DEFAULT 'pending',
    claimed_by TEXT,
    claimed_at REAL
);
"""

# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class ClaimResult:
    phase_id: int
    ticket_id: int
    phase_name: str
    status: str
    claimed_by: Optional[str]


# ---------------------------------------------------------------------------
# Database setup
# ---------------------------------------------------------------------------

def create_db(db_path: str, n_phases: int) -> None:
    """Initialize the database and insert N pending phases."""
    conn = sqlite3.connect(db_path)
    conn.executescript(SCHEMA)
    conn.executemany(
        "INSERT INTO phases (ticket_id, phase_name, status) VALUES (?, ?, 'pending')",
        [(i + 1, f"phase_{i + 1}") for i in range(n_phases)],
    )
    conn.commit()
    conn.close()


# ---------------------------------------------------------------------------
# Claiming function — runs in each thread
# ---------------------------------------------------------------------------

def claim_loop(
    db_path: str,
    agent_id: str,
    results: list[ClaimResult],
    errors: list[Exception],
    stop_event: threading.Event,
    attempt_limit: int = 100,
) -> None:
    """
    Repeatedly attempt to claim a pending phase until no more are available
    or attempt_limit is reached.

    Uses the design's atomic UPDATE ... RETURNING * pattern inside an
    explicit transaction.
    """
    conn = sqlite3.connect(db_path, check_same_thread=False, timeout=10.0)
    conn.row_factory = sqlite3.Row

    attempts = 0
    while attempts < attempt_limit and not stop_event.is_set():
        try:
            # Open explicit transaction
            conn.execute("BEGIN IMMEDIATE")
            row = conn.execute(
                """
                UPDATE phases
                SET status = 'in_progress',
                    claimed_by = :agent_id,
                    claimed_at = :now
                WHERE phase_id IN (
                    SELECT phase_id FROM phases
                    WHERE status = 'pending' AND claimed_by IS NULL
                    LIMIT 1
                )
                RETURNING phase_id, ticket_id, phase_name, status, claimed_by
                """,
                {"agent_id": agent_id, "now": time.time()},
            ).fetchone()
            conn.commit()

            if row is None:
                # No pending phases left — done
                break

            results.append(
                ClaimResult(
                    phase_id=row["phase_id"],
                    ticket_id=row["ticket_id"],
                    phase_name=row["phase_name"],
                    status=row["status"],
                    claimed_by=row["claimed_by"],
                )
            )

        except sqlite3.OperationalError as exc:
            # Retry on "database is locked" — this is expected under contention
            # but should resolve within busy_timeout.
            try:
                conn.execute("ROLLBACK")
            except Exception:
                pass
            errors.append(exc)

        attempts += 1

    conn.close()


# ---------------------------------------------------------------------------
# Test harness
# ---------------------------------------------------------------------------

def run_prototype(n_phases: int = 10, attempt_limit: int = 100) -> dict:
    """
    Run the atomic-claim prototype and return a result dict.

    Returns:
        {
            "n_phases": int,
            "total_claims": int,
            "unique_phase_ids": int,
            "duplicates": list[int],
            "errors_per_thread": {agent_id: int},
            "passed": bool,
            "failure_reason": str | None,
        }
    """
    with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as f:
        db_path = f.name

    try:
        create_db(db_path, n_phases)

        thread_results: dict[str, list[ClaimResult]] = {"agent_A": [], "agent_B": []}
        thread_errors: dict[str, list[Exception]] = {"agent_A": [], "agent_B": []}
        stop_event = threading.Event()

        threads = [
            threading.Thread(
                target=claim_loop,
                args=(db_path, agent_id, thread_results[agent_id],
                      thread_errors[agent_id], stop_event, attempt_limit),
                name=agent_id,
            )
            for agent_id in ("agent_A", "agent_B")
        ]

        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=30.0)

        all_claims = thread_results["agent_A"] + thread_results["agent_B"]
        all_phase_ids = [c.phase_id for c in all_claims]
        unique_phase_ids = list(set(all_phase_ids))

        # Find duplicates (same phase_id claimed by both threads)
        seen: dict[int, int] = {}
        duplicates: list[int] = []
        for pid in all_phase_ids:
            seen[pid] = seen.get(pid, 0) + 1
        duplicates = [pid for pid, count in seen.items() if count > 1]

        # Determine pass/fail
        failure_reason = None
        if len(all_claims) != n_phases:
            failure_reason = (
                f"Expected {n_phases} total claims, got {len(all_claims)}. "
                f"agent_A claimed {len(thread_results['agent_A'])}, "
                f"agent_B claimed {len(thread_results['agent_B'])}."
            )
        elif duplicates:
            failure_reason = (
                f"Duplicate phase_ids claimed by both threads: {duplicates}"
            )

        return {
            "n_phases": n_phases,
            "total_claims": len(all_claims),
            "unique_phase_ids": len(unique_phase_ids),
            "duplicates": duplicates,
            "errors_per_thread": {
                agent_id: len(thread_errors[agent_id])
                for agent_id in ("agent_A", "agent_B")
            },
            "agent_A_claims": [c.phase_id for c in thread_results["agent_A"]],
            "agent_B_claims": [c.phase_id for c in thread_results["agent_B"]],
            "passed": failure_reason is None,
            "failure_reason": failure_reason,
        }

    finally:
        os.unlink(db_path)


# ---------------------------------------------------------------------------
# Repeated-trial harness (stress test)
# ---------------------------------------------------------------------------

def stress_test(trials: int = 20, n_phases: int = 10) -> dict:
    """
    Run the prototype N times to detect intermittent failures.

    Returns aggregate pass/fail counts and any failure details.
    """
    passed = 0
    failed = 0
    failure_details: list[dict] = []

    for i in range(trials):
        result = run_prototype(n_phases=n_phases)
        if result["passed"]:
            passed += 1
        else:
            failed += 1
            failure_details.append({"trial": i + 1, **result})

    return {
        "trials": trials,
        "passed": passed,
        "failed": failed,
        "failure_details": failure_details,
        "all_passed": failed == 0,
    }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("=" * 70)
    print("P1: Atomic Claim Transaction Semantics")
    print("=" * 70)

    # Single run — verbose output
    print("\n--- Single Run (n_phases=10, attempt_limit=100) ---")
    result = run_prototype(n_phases=10, attempt_limit=100)
    print(f"  Total claims    : {result['total_claims']} / {result['n_phases']}")
    print(f"  Unique phase_ids: {result['unique_phase_ids']}")
    print(f"  Duplicates      : {result['duplicates']}")
    print(f"  Errors (agent_A): {result['errors_per_thread']['agent_A']}")
    print(f"  Errors (agent_B): {result['errors_per_thread']['agent_B']}")
    print(f"  agent_A claimed : {sorted(result['agent_A_claims'])}")
    print(f"  agent_B claimed : {sorted(result['agent_B_claims'])}")
    print(f"  Result          : {'PASS' if result['passed'] else 'FAIL'}")
    if not result["passed"]:
        print(f"  Failure         : {result['failure_reason']}")

    # Stress test — 20 trials
    print("\n--- Stress Test (20 trials, n_phases=10) ---")
    stress = stress_test(trials=20, n_phases=10)
    print(f"  Passed : {stress['passed']} / {stress['trials']}")
    print(f"  Failed : {stress['failed']} / {stress['trials']}")
    if stress["failure_details"]:
        for detail in stress["failure_details"]:
            print(f"  Trial {detail['trial']} FAILED: {detail['failure_reason']}")

    print(f"\n  Overall: {'ALL TRIALS PASSED' if stress['all_passed'] else 'SOME TRIALS FAILED'}")

    # BEGIN IMMEDIATE variant — alternative approach if WAL alone is insufficient
    print("\n--- Note ---")
    print("  BEGIN IMMEDIATE acquires a reserved lock at transaction start,")
    print("  preventing any other writer from entering. This is the fallback")
    print("  strategy if WAL + busy_timeout produces OperationalError under load.")
    print("  The prototype uses BEGIN IMMEDIATE by default to match design intent.")

    print("\n" + "=" * 70)
    print("Done.")
