import json
from pathlib import Path


class Motor:

    def __init__(self):

        base_dir = Path(__file__).resolve().parent.parent.parent
        data_file = base_dir / "data" / "uav_specs.json"

        try:
            with open(data_file, "r") as file:
                specs = json.load(file)
            self.max_power = specs.get("battery_max_power_kw", 250)
            self.efficiency = specs.get("motor_efficiency", 0.95)
        except Exception:
            self.max_power = 250
            self.efficiency = 0.95

        self.max_elec_power = float(self.max_power)
        self.max_shaft_power = self.max_elec_power * self.efficiency  # 237.5 kW
        self.name = "Electric Propulsion Motor"
        self.maximum_power = self.max_elec_power
        self.maximum_rpm = 12000

        self.running = False
        self.current_elec_power = 0.0
        self.current_shaft_power = 0.0
        self.current_rpm = 0
        self.current_thrust = 0.0

    def start(self):

        self.running = True
        print("\n[OK] Electric Motor Started")

    def stop(self):

        self.running = False
        self.current_elec_power = 0.0
        self.current_shaft_power = 0.0
        self.current_rpm = 0
        self.current_thrust = 0.0

        print("\n[STOP] Electric Motor Stopped")

    def get_shaft_power(self) -> float:
        """Returns mechanical shaft output power (kW)."""
        return self.current_shaft_power

    def is_motor_saturated(self) -> bool:
        """Returns True if electrical input power reaches maximum rated electrical power."""
        return self.current_elec_power >= self.max_elec_power

    def set_power(self, power_elec_kw):

        """Sets motor electrical input power and calculates mechanical shaft output power."""
        power_elec = float(power_elec_kw)

        if power_elec > self.max_elec_power:
            power_elec = self.max_elec_power

        if power_elec < 0.0:
            power_elec = 0.0

        self.current_elec_power = power_elec
        self.current_shaft_power = self.current_elec_power * self.efficiency

        self.current_rpm = int(
            (self.current_elec_power / self.max_elec_power) * self.maximum_rpm
        )

        self.current_thrust = round(self.current_elec_power * 4.5, 2)

    def display_information(self):

        print("\n========== MOTOR ==========")
        print(f"Motor            : {self.name}")
        print(f"Max Elec Power   : {self.max_elec_power:.1f} kW")
        print(f"Max Shaft Power  : {self.max_shaft_power:.1f} kW")
        print(f"Current Elec Pwr : {self.current_elec_power:.1f} kW")
        print(f"Current Shaft Pwr: {self.current_shaft_power:.1f} kW")
        print(f"RPM              : {self.current_rpm}")
        print(f"Efficiency       : {self.efficiency}")
        print("===========================")

    def motor_status(self):

        return {
            "elec_power": round(self.current_elec_power, 2),
            "shaft_power": round(self.current_shaft_power, 2),
            "rpm": self.current_rpm,
            "thrust": self.current_thrust
        }