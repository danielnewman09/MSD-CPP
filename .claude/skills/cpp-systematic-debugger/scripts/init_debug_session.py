#!/usr/bin/env python3
"""
Initialize a new C++ debug session from a DEBUG_TICKET.md file.

Usage:
    python init_debug_session.py                    # Uses ./DEBUG_TICKET.md
    python init_debug_session.py path/to/ticket.md  # Uses specified ticket
    python init_debug_session.py --dir /path/to/project
"""

import argparse
import os
from datetime import datetime
from pathlib import Path


INVESTIGATION_TEMPLATE = '''

---

## Investigation Log

### {timestamp} - Session Started

**Initial Focus:**
- [ ] Review header files for suspect classes
- [ ] Analyze implementation details  
- [ ] Check existing test coverage
- [ ] Run with sanitizers (ASan, UBSan)

**Observations:**
- [Add observations here]

---

## Hypotheses Tracker

### H1: [First Hypothesis]

**Status:** ⏳ Investigating

**Description:** [What might be causing the issue]

**Evidence For:**
- 

**Evidence Against:**
- 

**Tests to Verify:**
- [ ] 

**Conclusion:** Pending

---

## Test Results

[Tests will be documented here as they are created and run]

---

## Checkpoints

[Human checkpoints will be documented here]

---

## Resolution

[To be filled when issue is resolved]

---

## Session Notes

[Additional context and observations]
'''


def main():
    parser = argparse.ArgumentParser(
        description="Initialize a C++ debug session from DEBUG_TICKET.md"
    )
    parser.add_argument(
        "ticket",
        nargs="?",
        default="DEBUG_TICKET.md",
        help="Path to debug ticket file (default: DEBUG_TICKET.md)"
    )
    parser.add_argument(
        "--dir", "-d",
        default=".",
        help="Project directory (default: current directory)"
    )
    
    args = parser.parse_args()
    
    project_dir = Path(args.dir).resolve()
    ticket_path = project_dir / args.ticket
    
    if not ticket_path.exists():
        print(f"❌ Debug ticket not found: {ticket_path}")
        print()
        print("Create a DEBUG_TICKET.md file first. You can copy the template from:")
        print("   assets/DEBUG_TICKET_TEMPLATE.md")
        print()
        print("Or create a minimal ticket with:")
        print('   echo "# DEBUG TICKET\\n\\n## Issue Summary\\n[describe issue]" > DEBUG_TICKET.md')
        return 1
    
    # Create session directory
    sessions_dir = project_dir / ".debug-sessions"
    sessions_dir.mkdir(exist_ok=True)
    
    # Generate session ID and timestamp
    now = datetime.now()
    session_id = f"debug_{now.strftime('%Y%m%d_%H%M%S')}"
    timestamp = now.strftime("%Y-%m-%d %H:%M")
    
    # Read ticket content
    ticket_content = ticket_path.read_text()
    
    # Create debug log by combining ticket with investigation template
    log_content = ticket_content + INVESTIGATION_TEMPLATE.format(timestamp=timestamp)
    
    # Add session header at the top
    header = f"""<!-- 
Session ID: {session_id}
Started: {timestamp}
Status: 🔴 Active
-->

"""
    log_content = header + log_content
    
    # Write log file
    log_file = sessions_dir / f"{session_id}.md"
    log_file.write_text(log_content)
    
    print(f"✅ Debug session initialized!")
    print(f"   Session ID: {session_id}")
    print(f"   Log file: {log_file}")
    print(f"   From ticket: {ticket_path}")
    print()
    print("Next steps:")
    print("1. Review the suspect code listed in the ticket")
    print("2. Run initial analysis with sanitizers")
    print("3. Document observations in the Investigation Log")
    print("4. Form and test hypotheses")
    
    return 0


if __name__ == "__main__":
    exit(main())