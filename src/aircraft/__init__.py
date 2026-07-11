import json
from pathlib import Path


class Battery:

    def __init__(self):

        base_dir = Path(__file__).resolve().parent.parent.parent
        battery_file = base_dir / "data" / "battery_specs.json"

        with open(battery_file, "r") as file:
            specs = json.load(file)

        self.type = specs["battery_type"]
        self.capacity = specs["capacity_kwh"]
        self.voltage = specs["nominal_voltage_v"]
        self.max_power = specs["maximum_power_kw"]

        self.charge_efficiency = specs["charging_efficiency"]
        self.discharge_efficiency = specs["discharging_efficiency"]

        self.min_soc = specs["minimum_state_of_charge_percent"]
        self.max_soc = specs["maximum_state_of_charge_percent"]

        self.soc = specs["initial_state_of_charge_percent"]

        self.temperature = 25

        self.max_temperature = specs["maximum_temperature_c"]

        self.weight = specs["weight_kg"]

        self.cycle_life = specs["cycle_life"]

    def display_information(self):

        print("\n===== Battery Specifications =====")

        print(f"Battery Type : {self.type}")
        print(f"Capacity : {self.capacity} kWh")
        print(f"Voltage : {self.voltage} V")
        print(f"Maximum Power : {self.max_power} kW")
        print(f"State of Charge : {self.soc}%")
        print(f"Temperature : {self.temperature}°C")
        print(f"Weight : {self.weight} kg")
        print(f"Cycle Life : {self.cycle_life}")

        print("==============================")

    def discharge(self, percent):

        self.soc -= percent

        if self.soc < self.min_soc:
            self.soc = self.min_soc

    def charge(self, percent):

        self.soc += percent

        if self.soc > self.max_soc:
            self.soc = self.max_soc