"""Dashboard forwarding module for streamlit run uav_dashboard/dashboard.py compatibility."""
import sys
from pathlib import Path

# Forward to main app module
root_dir = Path(__file__).resolve().parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))

from app import main

if __name__ == "__main__":
    main()