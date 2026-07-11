import csv
from pathlib import Path


class Telemetry:

    def __init__(self):

        self.records = []

    def log(
        self,
        phase,
        mission_time,
        speed,
        altitude,
        engine_power,
        battery_power,
        battery_soc,
        fuel_remaining,
        motor_rpm,
    ):

        self.records.append({
            "Phase": phase,
            "Mission Time (min)": mission_time,
            "Speed (km/h)": speed,
            "Altitude (m)": altitude,
            "Engine Power (kW)": round(engine_power, 2),
            "Battery Power (kW)": round(battery_power, 2),
            "Battery SoC (%)": round(battery_soc, 2),
            "Fuel Remaining (kg)": round(fuel_remaining, 2),
            "Motor RPM": motor_rpm
        })

    def save_csv(self):

        output_folder = Path("logs")
        output_folder.mkdir(exist_ok=True)

        file_path = output_folder / "mission_log.csv"

        with open(file_path, "w", newline="") as csvfile:

            writer = csv.DictWriter(
                csvfile,
                fieldnames=self.records[0].keys()
            )

            writer.writeheader()

            writer.writerows(self.records)

        print(f"\n📄 Mission log saved to {file_path}")