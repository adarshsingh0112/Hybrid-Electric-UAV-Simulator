import json
from pathlib import Path


class Engine:

    def __init__(self):

        # Load engine specifications
        base_dir = Path(__file__).resolve().parent.parent.parent
        engine_file = base_dir / "data" / "engine_specs.json"

        with open(engine_file, "r") as file:
            specs = json.load(file)

        # Specifications
        self.name = specs["engine_name"]
        self.type = specs["engine_type"]

        self.max_power = specs["maximum_power_kw"]
        self.idle_power = specs["idle_power_kw"]

        self.fuel_type = specs["fuel_type"]

        self.sfc = specs["specific_fuel_consumption_kg_per_kwh"]

        self.max_rpm = specs["maximum_rpm"]

        self.generator_efficiency = specs["generator_efficiency"]

        self.weight = specs["weight_kg"]

        # Runtime variables
        self.running = False
        self.current_power = 0.0
        self.current_rpm = 0

        # Fuel tank
        # Fuel Tank Capacity
        self.fuel_capacity = 250.0   # kg
        self.fuel_remaining = self.fuel_capacity

    # -------------------------------------------------

    def start(self):

        self.running = True

        self.current_power = self.idle_power

        self.current_rpm = int(
            (self.idle_power / self.max_power) * self.max_rpm
        )

        print("\n✅ Engine Started")

    # -------------------------------------------------

    def stop(self):

        self.running = False

        self.current_power = 0

        self.current_rpm = 0

        print("\n🛑 Engine Stopped")

    # -------------------------------------------------

    def set_power(self, power):

        # Engine must be running
        if not self.running:

            self.current_power = 0
            self.current_rpm = 0
            return

        # No fuel -> no power
        if self.fuel_remaining <= 0:

            self.current_power = 0
            self.current_rpm = 0
            return

        # Clamp power
        power = max(
            self.idle_power,
            min(power, self.max_power)
        )

        self.current_power = power

        self.current_rpm = int(
            (power / self.max_power) * self.max_rpm
        )

    # -------------------------------------------------

    def consume_fuel(self, duration_min):

        if not self.running:
            return 0

        if self.fuel_remaining <= 0:

            self.fuel_remaining = 0
            self.stop()

            return 0

        fuel_used = (
            self.current_power *
            self.sfc *
            (duration_min / 60)
        )

        self.fuel_remaining -= fuel_used

        if self.fuel_remaining < 0:
            self.fuel_remaining = 0
            self.stop()

        return round(fuel_used, 3)

    # -------------------------------------------------

    def display_information(self):

        print("\n========== ENGINE ==========")

        print(f"Engine           : {self.name}")
        print(f"Type             : {self.type}")
        print(f"Maximum Power    : {self.max_power} kW")
        print(f"Current Power    : {self.current_power:.1f} kW")
        print(f"RPM              : {self.current_rpm}")
        print(f"Fuel Capacity    : {self.fuel_capacity:.2f} kg")
        print(f"Fuel Remaining   : {self.fuel_remaining:.2f} kg")

        print("============================")

    # -------------------------------------------------

    def engine_status(self):

        return {

            "power": round(self.current_power, 2),

            "rpm": self.current_rpm,

            "fuel": round(self.fuel_remaining, 2)

        }