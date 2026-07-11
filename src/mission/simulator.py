import time

from telemetry.telemetry import Telemetry
from dashboard.dashboard import Dashboard


class MissionSimulator:

    def __init__(self, mission, power_manager):

        self.mission = mission
        self.power_manager = power_manager

        # Mission timer
        self.mission_time = 0

        # Telemetry logger
        self.telemetry = Telemetry()

        # Live dashboard
        self.dashboard = Dashboard()

    # ---------------------------------------------------------

    def run(self):

        print("\n" + "=" * 60)
        print("MISSION START")
        print("=" * 60)

        for phase in self.mission.phases:

            phase_name = phase["name"]
            duration = phase["duration_min"]

            # Required propulsion power
            required_power = (
                phase["power_requirement_percent"] / 100
            ) * self.power_manager.motor.max_power

            # Hybrid Energy Management
            engine_power, battery_power = self.power_manager.execute(
                phase_name,
                required_power,
                duration
            )

            # Update mission timer
            self.mission_time += duration

            # ---------------- Mission Information ----------------

            print(f"\n🚀 {phase_name}")
            print("-" * 50)

            print(f"Mission Time     : {self.mission_time} min")
            print(f"Duration         : {duration} min")

            print(
                f"Speed            : "
                f"{phase['target_speed_kmph']} km/h"
            )

            print(
                f"Altitude         : "
                f"{phase['target_altitude_m']} m"
            )

            print(f"Required Power   : {required_power:.1f} kW")
            print(f"Engine Power     : {engine_power:.1f} kW")
            print(f"Battery Power    : {battery_power:.1f} kW")

            print(
                f"Battery SoC      : "
                f"{self.power_manager.battery.soc:.1f}%"
            )

            print(
                f"Battery Temp     : "
                f"{self.power_manager.battery.temperature:.1f} °C"
            )

            print(
                f"Fuel Remaining   : "
                f"{self.power_manager.engine.fuel_remaining:.2f} kg"
            )

            print(
                f"Motor RPM        : "
                f"{self.power_manager.motor.current_rpm}"
            )

            print(
                f"Motor Thrust     : "
                f"{self.power_manager.motor.current_thrust:.2f} N"
            )

            # ---------------- Dashboard ----------------

            self.dashboard.display(

                phase_name,

                self.mission_time,

                self.power_manager.engine,

                self.power_manager.battery,

                self.power_manager.motor

            )

            # ---------------- Telemetry ----------------

            self.telemetry.log(

                phase_name,

                self.mission_time,

                phase["target_speed_kmph"],

                phase["target_altitude_m"],

                engine_power,

                battery_power,

                self.power_manager.battery.soc,

                self.power_manager.engine.fuel_remaining,

                self.power_manager.motor.current_rpm

            )

            time.sleep(1)

        # Save mission data
        self.telemetry.save_csv()

        print("\n" + "=" * 60)
        print("MISSION COMPLETED SUCCESSFULLY")
        print("=" * 60)