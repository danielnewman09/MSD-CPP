"""
pytest configuration for workflow engine tests.

Adds the scripts/ directory to sys.path so that
'from workflow.engine.xxx import ...' works correctly.
"""

import sys
from pathlib import Path

# Ensure scripts/ is on the path (workflow package lives at scripts/workflow/)
scripts_dir = Path(__file__).parent.parent.parent
if str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))
