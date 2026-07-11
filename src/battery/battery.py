import json
from pathlib import Path


class Battery:

    def __init__(self):

        base_dir = Path(__file__).resolve().parent.parent.parent
        battery_file = base_dir / "data" / "battery_specs.json"

        with open(battery_file, "r") as file:
            specs = json.load(file)

        # Specifications
        self.type = specs["battery_type"]
        self.capacity_kwh = specs["capacity_kwh"]
        self.voltage = specs["nominal_voltage_v"]
        self.max_power = specs["maximum_power_kw"]

        self.charge_efficiency = specs["charging_efficiency"]
        self.discharge_efficiency = specs["discharging_efficiency"]

        self.min_soc = specs["minimum_state_of_charge_percent"]
        self.max_soc = specs["maximum_state_of_charge_percent"]

        self.soc = specs["initial_state_of_charge_percent"]

        self.temperature = 25.0
        self.max_temperature = specs["maximum_temperature_c"]

        self.weight = specs["weight_kg"]
        self.cycle_life = specs["cycle_life"]

        # Runtime Variables
        self.energy_remaining = (
            self.capacity_kwh *
            self.soc / 100
        )

        self.health = 100.0

    # -----------------------------------------------------

    def display_information(self):

        print("\n========== BATTERY ==========")

        print(f"Type              : {self.type}")
        print(f"Capacity          : {self.capacity_kwh:.2f} kWh")
        print(f"Energy Remaining  : {self.energy_remaining:.2f} kWh")
        print(f"SoC               : {self.soc:.1f}%")
        print(f"Voltage           : {self.voltage} V")
        print(f"Temperature       : {self.temperature:.1f} °C")
        print(f"Health            : {self.health:.1f}%")

        print("=============================")

    # -----------------------------------------------------

    def consume_energy(self, battery_power, duration):

        duration_hr = duration / 60

        energy_used = (
            battery_power *
            duration_hr
        ) / self.discharge_efficiency

        # Hybrid UAV
        hybrid_factor = 0.45

        energy_used *= hybrid_factor

        self.energy_remaining -= energy_used

        minimum_energy = (
            self.capacity_kwh *
            self.min_soc / 100
        )

        if self.energy_remaining < minimum_energy:
            self.energy_remaining = minimum_energy

        self.soc = (
            self.energy_remaining /
            self.capacity_kwh
        ) * 100

        self.temperature += battery_power * 0.003

        if self.temperature > self.max_temperature:
            self.temperature = self.max_temperature

    # -----------------------------------------------------

    def recharge(self, power_kw, duration_min):

        duration_hr = duration_min / 60

        energy_added = (
            power_kw *
            duration_hr *
            self.charge_efficiency
        )

        self.energy_remaining += energy_added

        if self.energy_remaining > self.capacity_kwh:
            self.energy_remaining = self.capacity_kwh

        self.soc = (
            self.energy_remaining /
            self.capacity_kwh
        ) * 100

    # -----------------------------------------------------

    def cool_down(self):

        if self.temperature > 25:
            self.temperature -= 1

    # -----------------------------------------------------

    def battery_status(self):

        return {

            "soc": round(self.soc, 2),

            "energy": round(self.energy_remaining, 2),

            "temperature": round(self.temperature, 2),

            "health": round(self.health, 2)

        }