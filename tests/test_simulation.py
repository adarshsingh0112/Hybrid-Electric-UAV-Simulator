import pytest
import sys
import pandas as pd
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parent.parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controller.hybrid_uav import HybridUAV


def test_full_mission_simulation_integration():
    """Verify end-to-end UAV initialization and simulation execution."""
    uav = HybridUAV()
    uav.initialize()
    uav.fly()

    log_file = Path(__file__).resolve().parent.parent / "logs" / "mission_log.csv"
    assert log_file.exists(), "Telemetry mission_log.csv was not created!"

    df = pd.read_csv(log_file)
    assert len(df) >= 21000, f"Expected at least 21000 time-step records in log, found {len(df)}"

    expected_columns = [
        "Phase", "Mission Time (min)", "Speed (km/h)", "Altitude (m)",
        "Engine Power (kW)", "Battery Power (kW)", "Battery SoC (%)",
        "Fuel Remaining (kg)", "Motor RPM"
    ]
    for col in expected_columns:
        assert col in df.columns, f"Missing required telemetry column: {col}"

    # Verify no negative fuel and SoC bounds in log
    assert (df["Fuel Remaining (kg)"] >= 0).all()
    assert (df["Battery SoC (%)"] >= 0).all()
    assert (df["Battery SoC (%)"] <= 100).all()
