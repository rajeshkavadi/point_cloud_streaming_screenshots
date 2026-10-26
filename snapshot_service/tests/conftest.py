import sys
from pathlib import Path

# Make the project root importable so `pc_snapshot` and `tests` resolve.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
