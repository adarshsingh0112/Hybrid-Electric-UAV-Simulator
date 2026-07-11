import json
from pathlib import Path


class Mission:

    def __init__(self):

        base_dir = Path(__file__).resolve().parent.parent.parent

        mission_file = base_dir / "data" / "mission_profile.json"

        with open(mission_file, "r") as file:
            mission = json.load(file)

        self.name = mission["mission_name"]
        self.phases = mission["phases"]

    def display_mission(self):

        print("\n========== MISSION ==========")
        print(f"Mission : {self.name}")
        print("=============================")

        for phase in self.phases:

            print(f"\nPhase : {phase['name']}")
            print(f"Duration : {phase['duration_min']} min")
            print(f"Speed : {phase['target_speed_kmph']} km/h")
            print(f"Altitude : {phase['target_altitude_m']} m")
            print(f"Power : {phase['power_requirement_percent']} %")