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
        self.bsfc_g_per_kwh = self.sfc * 1000.0  # Explicit g/kWh (e.g. 180 g/kWh)

        self.max_rpm = specs["maximum_rpm"]

        self.generator_efficiency = specs["generator_efficiency"]
        self.generator_max_elec_kw = specs.get(
            "generator_max_electrical_kw",
            self.max_power * self.generator_efficiency
        )  # 279.0 kW

        self.weight = specs["weight_kg"]

        # Runtime variables
        self.running = False
        self.current_power = 0.0  # Mechanical shaft power (kW)
        self.current_rpm = 0

        # Fuel tank
        self.fuel_capacity = float(specs.get("fuel_capacity_kg", 250.0))
        self.fuel_remaining = self.fuel_capacity

    # -------------------------------------------------

    def start(self):

        self.running = True

        self.current_power = self.idle_power

        self.current_rpm = int(
            (self.idle_power / self.max_power) * self.max_rpm
        )

        print("\n[OK] Engine Started")

    # -------------------------------------------------

    def stop(self):

        self.running = False

        self.current_power = 0.0

        self.current_rpm = 0

        print("\n[STOP] Engine Stopped")

    # -------------------------------------------------

    def get_electrical_power(self):

        """Returns electrical power (kW) generated from mechanical shaft power,
        clamped to the configurable generator rated electrical limit.
        """
        if not self.running or self.fuel_remaining <= 0:
            return 0.0

        raw_elec = self.current_power * self.generator_efficiency
        gen_elec = min(raw_elec, self.generator_max_elec_kw)
        return gen_elec

    def is_generator_saturated(self) -> bool:
        """Returns True if generator electrical output hits rated limit."""
        if not self.running or self.fuel_remaining <= 0:
            return False
        return (self.current_power * self.generator_efficiency) >= self.generator_max_elec_kw

    # -------------------------------------------------

    def set_power(self, power):

        # Engine must be running
        if not self.running:

            self.current_power = 0.0
            self.current_rpm = 0
            return

        # No fuel -> no power
        if self.fuel_remaining <= 0:

            self.current_power = 0.0
            self.current_rpm = 0
            return

        # Clamp mechanical power
        power = max(
            self.idle_power,
            min(power, self.max_power)
        )

        self.current_power = float(power)

        self.current_rpm = int(
            (self.current_power / self.max_power) * self.max_rpm
        )

    # -------------------------------------------------

    def consume_fuel(self, duration, is_seconds: bool = False):

        """Calculates fuel consumption using dimensionally explicit BSFC formula:
        fuel_used_kg = (power_kw * bsfc_g_per_kwh * dt_seconds) / 3_600_000
        """
        if not self.running:
            return 0.0

        if self.fuel_remaining <= 0:

            self.fuel_remaining = 0.0
            self.stop()

            return 0.0

        dt_seconds = float(duration) if is_seconds else float(duration) * 60.0

        # Dimensionally explicit BSFC fuel mass flow calculation (kg)
        fuel_used = (
            self.current_power *
            self.bsfc_g_per_kwh *
            dt_seconds
        ) / 3_600_000.0

        self.fuel_remaining -= fuel_used

        if self.fuel_remaining <= 0:
            self.fuel_remaining = 0.0
            self.stop()

        return round(fuel_used, 4)

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