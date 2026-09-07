import pandas as pd
import numpy as np
import streamlit as st
from pathlib import Path
from typing import Tuple, Dict, Any, Optional

REQUIRED_COLUMNS = [
    "Time (s)",
    "Mission Time (min)",
    "Phase",
    "Altitude (m)",
    "Airspeed (m/s)",
    "Aircraft Mass (kg)",
    "Fuel Remaining (kg)",
    "Battery SoC (%)",
    "Engine Power (kW)",
    "Generator Power (kW)",
    "Battery Power (kW)",
    "Required Elec Power (kW)",
    "Motor Elec Power (kW)",
    "Motor Shaft Power (kW)",
    "Power Deficit (kW)",
    "Is Power Limited",
]


@st.cache_data(show_spinner=False)
def load_telemetry(file_path: str, mtime: float) -> pd.DataFrame:
    """Loads telemetry CSV file with caching based on file modification timestamp."""
    path = Path(file_path)
    if not path.exists():
        raise FileNotFoundError(f"Telemetry log file not found at: {file_path}")

    df = pd.read_csv(path)
    if df.empty:
        raise ValueError("Telemetry log file is empty.")

    return df


def validate_telemetry(df: pd.DataFrame) -> Tuple[bool, Optional[str]]:
    """Validates telemetry structure, columns, values, and timestamp monotonicity.

    Returns:
        Tuple[bool, str]: (is_valid, error_message_if_invalid)
    """
    if df is None or df.empty:
        return False, "Telemetry data is empty or invalid."

    # 1. Column existence check
    missing_cols = [col for col in REQUIRED_COLUMNS if col not in df.columns]
    if missing_cols:
        return False, f"Missing required telemetry columns: {', '.join(missing_cols)}"

    # 2. Check for NaN values in critical numeric columns
    numeric_cols = [
        "Time (s)", "Altitude (m)", "Airspeed (m/s)", "Aircraft Mass (kg)",
        "Fuel Remaining (kg)", "Battery SoC (%)", "Power Deficit (kW)"
    ]
    nan_cols = [col for col in numeric_cols if df[col].isnull().any()]
    if nan_cols:
        return False, f"Telemetry contains NaN values in columns: {', '.join(nan_cols)}"

    # 3. Check time monotonicity
    time_series = df["Time (s)"].values
    if not np.all(np.diff(time_series) >= 0):
        return False, "Telemetry time series timestamps are not monotonically increasing."

    return True, None


def calculate_dashboard_metrics(df: pd.DataFrame) -> Dict[str, Any]:
    """Computes aggregate mission statistics and power deficit metrics from telemetry dataframe.

    Deficit energy is calculated dynamically from actual time-series step dt:
    E_deficit_kWh = sum(power_deficit_kw[i] * dt[i]) / 3600
    """
    if df is None or df.empty:
        return {}

    total_rows = len(df)
    times_s = df["Time (s)"].values
    duration_s = float(times_s[-1]) if total_rows > 0 else 0.0
    duration_min = duration_s / 60.0

    initial_mass = float(df["Aircraft Mass (kg)"].iloc[0])
    final_mass = float(df["Aircraft Mass (kg)"].iloc[-1])

    initial_fuel = float(df["Fuel Remaining (kg)"].iloc[0])
    final_fuel = float(df["Fuel Remaining (kg)"].iloc[-1])
    fuel_consumed = max(0.0, initial_fuel - final_fuel)

    initial_soc = float(df["Battery SoC (%)"].iloc[0])
    final_soc = float(df["Battery SoC (%)"].iloc[-1])

    max_req_power = float(df["Required Elec Power (kW)"].max())
    max_gen_power = float(df["Generator Power (kW)"].max())
    max_bat_power = float(df["Battery Power (kW)"].max())

    # Dynamic step dt power deficit calculation
    power_deficits = df["Power Deficit (kW)"].values
    is_limited = df["Is Power Limited"].values

    power_limited_seconds = 0.0
    total_deficit_energy_kwh = 0.0
    max_deficit_kw = float(power_deficits.max()) if total_rows > 0 else 0.0

    if total_rows > 1:
        dts = np.diff(times_s)
        # Pad dts for first element using first dt or 1.0s
        first_dt = dts[0] if len(dts) > 0 else 1.0
        step_dts = np.insert(dts, 0, first_dt)

        deficit_mask = (power_deficits > 1e-4) | (is_limited == True)
        power_limited_seconds = float(np.sum(step_dts[deficit_mask]))
        total_deficit_energy_kwh = float(np.sum((power_deficits[deficit_mask] * step_dts[deficit_mask]) / 3600.0))

    # Determine explicit Mission Status
    if final_fuel < 0 or final_soc < 0:
        mission_status = "MISSION INFEASIBLE / ERROR"
        status_color = "red"
    elif power_limited_seconds > 1e-3 or total_deficit_energy_kwh > 1e-4:
        mission_status = "MISSION COMPLETE — POWER LIMITED"
        status_color = "orange"
    else:
        mission_status = "MISSION SUCCESS"
        status_color = "green"

    return {
        "mission_status": mission_status,
        "status_color": status_color,
        "duration_s": round(duration_s, 1),
        "duration_min": round(duration_min, 2),
        "total_rows": total_rows,
        "initial_mass_kg": round(initial_mass, 2),
        "final_mass_kg": round(final_mass, 2),
        "initial_fuel_kg": round(initial_fuel, 2),
        "final_fuel_kg": round(final_fuel, 2),
        "fuel_consumed_kg": round(fuel_consumed, 4),
        "initial_soc_percent": round(initial_soc, 2),
        "final_soc_percent": round(final_soc, 2),
        "max_req_power_kw": round(max_req_power, 2),
        "max_gen_power_kw": round(max_gen_power, 2),
        "max_bat_power_kw": round(max_bat_power, 2),
        "power_limited_seconds": round(power_limited_seconds, 2),
        "total_deficit_energy_kwh": round(total_deficit_energy_kwh, 4),
        "max_deficit_kw": round(max_deficit_kw, 2),
    }
