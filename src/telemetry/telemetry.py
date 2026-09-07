import csv
from pathlib import Path
from typing import List, Dict, Any, Optional

from mission.state import SimulationState


class Telemetry:

    def __init__(self):
        self.records: List[Dict[str, Any]] = []
        self.states: List[SimulationState] = []

    def log(
        self,
        phase: str,
        mission_time: float,
        speed: float,
        altitude: float,
        engine_power: float,
        battery_power: float,
        battery_soc: float,
        fuel_remaining: float,
        motor_rpm: int
    ):
        """Legacy phase-level logging method for backward compatibility."""
        self.records.append({
            "Phase": phase,
            "Mission Time (min)": round(mission_time, 2),
            "Speed (km/h)": round(speed, 2),
            "Altitude (m)": round(altitude, 2),
            "Engine Power (kW)": round(engine_power, 2),
            "Battery Power (kW)": round(battery_power, 2),
            "Battery SoC (%)": round(battery_soc, 2),
            "Fuel Remaining (kg)": round(fuel_remaining, 2),
            "Motor RPM": motor_rpm
        })

    def log_state(self, state: SimulationState):
        """Logs continuous simulation state record."""
        self.states.append(state)
        self.records.append(state.to_dict())

    def get_summary(self) -> Dict[str, Any]:
        """Calculates aggregate simulation statistics."""
        if not self.records:
            return {}

        if self.states:
            total_duration_s = self.states[-1].time_s
            initial_fuel = self.states[0].remaining_fuel_kg
            final_fuel = self.states[-1].remaining_fuel_kg
            fuel_consumed = max(0.0, initial_fuel - final_fuel)

            initial_soc = self.states[0].battery_soc_percent
            final_soc = self.states[-1].battery_soc_percent

            power_limited_seconds = 0.0
            total_deficit_energy_kwh = 0.0
            max_deficit_kw = 0.0

            for i in range(len(self.states)):
                dt = self.states[i].time_s - (self.states[i - 1].time_s if i > 0 else 0.0)
                st = self.states[i]
                if st.is_power_limited or st.power_deficit_kw > 1e-4:
                    power_limited_seconds += dt
                    total_deficit_energy_kwh += (st.power_deficit_kw * dt) / 3600.0
                    if st.power_deficit_kw > max_deficit_kw:
                        max_deficit_kw = st.power_deficit_kw

            return {
                "total_flight_time_s": round(total_duration_s, 2),
                "total_flight_time_min": round(total_duration_s / 60.0, 2),
                "initial_fuel_kg": round(initial_fuel, 2),
                "final_fuel_kg": round(final_fuel, 2),
                "fuel_consumed_kg": round(fuel_consumed, 4),
                "initial_soc_percent": round(initial_soc, 2),
                "final_soc_percent": round(final_soc, 2),
                "power_limited_seconds": round(power_limited_seconds, 2),
                "total_deficit_energy_kwh": round(total_deficit_energy_kwh, 4),
                "max_power_deficit_kw": round(max_deficit_kw, 2),
                "record_count": len(self.records),
            }
        else:
            first = self.records[0]
            last = self.records[-1]
            return {
                "total_flight_time_min": last.get("Mission Time (min)", 0),
                "record_count": len(self.records)
            }

    def save_csv(self, filename: str = "mission_log.csv"):
        """Saves telemetry records to CSV file in logs/ directory."""
        if not self.records:
            print("No telemetry data to save.")
            return

        logs_folder = Path("logs")
        logs_folder.mkdir(exist_ok=True)

        file_path = logs_folder / filename

        with open(file_path, "w", newline="") as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=list(self.records[0].keys())
            )

            writer.writeheader()
            writer.writerows(self.records)

        print(f"\n[LOG] Mission log saved to: {file_path} ({len(self.records)} rows)")