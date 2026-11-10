"""
Root test runner proxy for tests/test_mission_rerouting_api.py
"""
import pytest
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tests.test_mission_rerouting_api import *

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
