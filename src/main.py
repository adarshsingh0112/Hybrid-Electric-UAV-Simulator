import json
from pathlib import Path

from controller.hybrid_uav import HybridUAV

BASE_DIR = Path(__file__).resolve().parent.parent
DATA_DIR = BASE_DIR / "data"

FILES = [
    "uav_specs.json",
    "battery_specs.json",
    "engine_specs.json",
    "mission_profile.json"
]


def check_configuration():
    print("=" * 60)
    print("      Hybrid Electric UAV Simulator")
    print("=" * 60)

    all_loaded = True

    for file_name in FILES:
        try:
            with open(DATA_DIR / file_name, "r") as f:
                json.load(f)

            print(f"[OK] Loaded: {file_name}")

        except Exception as e:
            all_loaded = False
            print(f"[ERROR] Error loading {file_name}")
            print(e)

    print("=" * 60)

    return all_loaded


def main():

    if not check_configuration():
        print("\n[ERROR] Simulator cannot start due to configuration errors.")
        return

    print("\n[START] Starting Hybrid UAV...\n")

    # Create UAV
    uav = HybridUAV()

    # Initialize all subsystems
    uav.initialize()

    # Execute the complete mission
    uav.fly()

    print("\n[DONE] Simulation Finished Successfully.")


if __name__ == "__main__":
    main()