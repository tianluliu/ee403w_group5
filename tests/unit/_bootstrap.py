"""Make ``src/`` importable so tests run with no PYTHONPATH and no installation."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
CONFIG_DIR = REPO_ROOT / "config"

if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
