import pytest
import sys
import os
import pandas as pd
import numpy as np
from pathlib import Path

# Add root directory first to sys.path so root uav_dashboard package takes precedence
root_path = Path(__file__).resolve().parent.parent
if str(root_path) in sys.path:
    sys.path.remove(str(root_path))
sys.path.insert(0, str(root_path))

src_path = root_path / "src"
if str(src_path) not in sys.path:
    sys.path.append(str(src_path))

from uav_dashboard.utils.data_loader import (
    load_telemetry,
    validate_telemetry,
    calculate_dashboard_metrics,
    REQUIRED_COLUMNS
)


def create_sample_telemetry_df(power_deficit_kw: float = 0.0, dt_s: float = 1.0) -> pd.DataFrame:
    """Helper to generate a valid test telemetry DataFrame."""
    steps = 100
    times_s = np.arange(1, steps + 1) * dt_s
    times_min = times_s / 60.0

    return pd.DataFrame({
        "Time (s)": times_s,
        "Mission Time (min)": times_min,
        "Phase": ["Cruise"] * steps,
        "Altitude (m)": [5000.0] * steps,
        "Airspeed (m/s)": [69.44] * steps,
        "Aircraft Mass (kg)": np.linspace(1620.0, 1600.0, steps),
        "Fuel Remaining (kg)": np.linspace(250.0, 230.0, steps),
        "Battery SoC (%)": np.linspace(100.0, 80.0, steps),
        "Engine Power (kW)": [150.0] * steps,
        "Generator Power (kW)": [139.5] * steps,
        "Battery Power (kW)": [20.0] * steps,
        "Required Elec Power (kW)": [159.5] * steps,
        "Motor Elec Power (kW)": [159.5] * steps,
        "Motor Shaft Power (kW)": [151.5] * steps,
        "Power Deficit (kW)": [power_deficit_kw] * steps,
        "Is Power Limited": [power_deficit_kw > 0] * steps,
    })


def test_telemetry_csv_loading(tmp_path):
    df_sample = create_sample_telemetry_df()
    csv_file = tmp_path / "test_log.csv"
    df_sample.to_csv(csv_file, index=False)

    mtime = os.path.getmtime(csv_file)
    df_loaded = load_telemetry(str(csv_file), mtime)

    assert len(df_loaded) == 100
    assert "Aircraft Mass (kg)" in df_loaded.columns


def test_telemetry_validation_valid():
    df_sample = create_sample_telemetry_df()
    is_valid, err = validate_telemetry(df_sample)
    assert is_valid is True
    assert err is None


def test_telemetry_validation_missing_columns():
    df_sample = create_sample_telemetry_df().drop(columns=["Battery SoC (%)"])
    is_valid, err = validate_telemetry(df_sample)
    assert is_valid is False
    assert "Missing required telemetry columns" in err


def test_dashboard_metrics_calculation():
    df_sample = create_sample_telemetry_df()
    metrics = calculate_dashboard_metrics(df_sample)

    assert metrics["mission_status"] == "MISSION SUCCESS"
    assert metrics["initial_mass_kg"] == 1620.0
    assert metrics["final_mass_kg"] == 1600.0
    assert metrics["fuel_consumed_kg"] == 20.0
    assert metrics["final_soc_percent"] == 80.0
    assert metrics["max_req_power_kw"] == 159.5
    assert metrics["power_limited_seconds"] == 0.0
    assert metrics["total_deficit_energy_kwh"] == 0.0


def test_power_deficit_metrics_calculation():
    # Deficit of 50 kW for 100 steps of 0.5s = 50s total deficit time
    # Energy = (50 kW * 50 s) / 3600 = 0.6944 kWh
    df_deficit = create_sample_telemetry_df(power_deficit_kw=50.0, dt_s=0.5)
    metrics = calculate_dashboard_metrics(df_deficit)

    assert metrics["mission_status"] == "MISSION COMPLETE — POWER LIMITED"
    assert metrics["power_limited_seconds"] == 50.0
    assert pytest.approx(metrics["total_deficit_energy_kwh"], abs=1e-3) == (50.0 * 50.0) / 3600.0
    assert metrics["max_deficit_kw"] == 50.0


def test_mission_status_classification():
    # Test INFEASIBLE status when fuel is negative
    df_error = create_sample_telemetry_df()
    df_error.loc[df_error.index[-1], "Fuel Remaining (kg)"] = -5.0
    metrics_error = calculate_dashboard_metrics(df_error)
    assert metrics_error["mission_status"] == "MISSION INFEASIBLE / ERROR"
