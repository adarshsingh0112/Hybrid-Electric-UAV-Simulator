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
        self.capacity_kwh = float(specs["capacity_kwh"])
        self.voltage = specs["nominal_voltage_v"]
        self.max_discharge_power = float(specs.get("maximum_power_kw", 250.0))
        self.max_power = self.max_discharge_power
        self.max_charge_power = float(specs.get("maximum_charge_power_kw", 100.0))

        self.charge_efficiency = float(specs["charging_efficiency"])
        self.discharge_efficiency = float(specs["discharging_efficiency"])

        # Configurable mission reserve SOC percentage (default 20.0%)
        self.min_soc = float(specs.get("minimum_state_of_charge_percent", 20.0))
        self.min_soc_fraction = self.min_soc / 100.0
        self.max_soc = float(specs.get("maximum_state_of_charge_percent", 100.0))

        self.soc = float(specs.get("initial_state_of_charge_percent", 100.0))

        self.temperature = 25.0
        self.max_temperature = specs["maximum_temperature_c"]

        self.weight = specs["weight_kg"]
        self.cycle_life = specs["cycle_life"]

        # Runtime Variables
        self.energy_remaining = (
            self.capacity_kwh *
            self.soc / 100.0
        )

        self.health = 100.0

    # -----------------------------------------------------

    def get_available_discharge_power(self) -> float:
        """Returns maximum available discharge power (kW).
        If SoC reaches or drops below configurable mission reserve (min_soc), available power is 0.
        """
        if self.soc <= self.min_soc:
            return 0.0
        return self.max_discharge_power

    def get_available_charge_power(self) -> float:
        """Returns maximum available charge power (kW).
        If SoC reaches 100%, available charge power is 0.
        """
        if self.soc >= self.max_soc:
            return 0.0
        return self.max_charge_power

    def display_information(self):

        print("\n========== BATTERY ==========")

        print(f"Type              : {self.type}")
        print(f"Capacity          : {self.capacity_kwh:.2f} kWh")
        print(f"Energy Remaining  : {self.energy_remaining:.2f} kWh")
        print(f"SoC               : {self.soc:.1f}% (Reserve: {self.min_soc:.1f}%)")
        print(f"Max Discharge Pwr : {self.max_discharge_power:.1f} kW")
        print(f"Max Charge Pwr    : {self.max_charge_power:.1f} kW")
        print(f"Voltage           : {self.voltage} V")
        print(f"Temperature       : {self.temperature:.1f} °C")
        print(f"Health            : {self.health:.1f}%")

        print("=============================")

    # -----------------------------------------------------

    def consume_energy(self, battery_power, duration, is_seconds: bool = False):

        """Discharges battery accounting for power limits, usable reserve energy,
        and discharge efficiency.
        """
        duration_hr = float(duration) / 3600.0 if is_seconds else float(duration) / 60.0

        # 1. Enforce reserve energy check
        if self.soc <= self.min_soc:
            return 0.0

        # 2. Enforce maximum discharge power limit
        power_kw = min(float(battery_power), self.max_discharge_power)
        if power_kw <= 0.0:
            return 0.0

        # 3. Calculate physical energy drawn from battery storage
        energy_requested = (power_kw * duration_hr) / self.discharge_efficiency

        # 4. Enforce usable energy bound (reserve energy protected)
        reserve_energy = self.capacity_kwh * self.min_soc_fraction
        usable_energy = max(0.0, self.energy_remaining - reserve_energy)

        energy_actual_drawn = min(energy_requested, usable_energy)

        self.energy_remaining -= energy_actual_drawn

        self.soc = (self.energy_remaining / self.capacity_kwh) * 100.0
        self.soc = max(self.min_soc, min(self.max_soc, self.soc))

        self.temperature += power_kw * 0.003
        if self.temperature > self.max_temperature:
            self.temperature = self.max_temperature

        delivered_power_kw = (energy_actual_drawn * self.discharge_efficiency) / duration_hr if duration_hr > 0 else 0.0
        return delivered_power_kw

    # -----------------------------------------------------

    def recharge(self, power_kw, duration, is_seconds: bool = False):

        """Recharges battery accounting for max charge power limit, remaining capacity,
        and charging efficiency.
        """
        duration_hr = float(duration) / 3600.0 if is_seconds else float(duration) / 60.0

        if self.soc >= self.max_soc:
            return 0.0

        # Enforce maximum charge power limit
        charge_power = min(float(power_kw), self.max_charge_power)
        if charge_power <= 0.0:
            return 0.0

        energy_added = charge_power * duration_hr * self.charge_efficiency
        remaining_capacity = (self.capacity_kwh * self.max_soc / 100.0) - self.energy_remaining

        actual_energy_added = min(energy_added, remaining_capacity)
        self.energy_remaining += actual_energy_added

        self.soc = (self.energy_remaining / self.capacity_kwh) * 100.0
        self.soc = max(0.0, min(self.max_soc, self.soc))

        absorbed_power_kw = (actual_energy_added / self.charge_efficiency) / duration_hr if duration_hr > 0 else 0.0
        return absorbed_power_kw

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