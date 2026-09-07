import json
import math
from pathlib import Path

from environment.atmosphere import ISAAtmosphere


class Aircraft:
    G = 9.80665  # Gravitational acceleration (m/s^2)

    def __init__(self):
        base_dir = Path(__file__).resolve().parent.parent.parent
        data_file = base_dir / "data" / "uav_specs.json"

        with open(data_file, "r") as file:
            specs = json.load(file)

        self.name = specs["uav_name"]
        self.mtow = specs["maximum_takeoff_weight_kg"]
        self.empty_weight = specs["empty_weight_kg"]
        self.payload = specs["payload_weight_kg"]
        self.fuel = specs["fuel_capacity_kg"]
        self.battery_weight = float(specs.get("battery_weight_kg", 520.0))
        self.battery_capacity = specs["battery_capacity_kwh"]
        self.cruise_speed = specs["cruise_speed_kmph"]
        self.max_speed = specs["maximum_speed_kmph"]
        self.ceiling = specs["service_ceiling_m"]
        self.range = specs["maximum_range_km"]
        self.endurance = specs["target_endurance_hr"]

        # Aerodynamic Parameters
        self.wing_span = specs["wing_span_m"]
        self.wing_area = specs["wing_area_sq_m"]
        self.aspect_ratio = (self.wing_span ** 2) / self.wing_area  # ~11.636
        self.oswald_eff = specs.get("oswald_efficiency", 0.82)
        self.cd0 = specs.get("cd0", 0.025)
        self.cl_max = specs.get("cl_max", 1.5)
        self.lift_drag = specs.get("lift_to_drag_ratio", 18)

        # Efficiency Chain Parameters
        self.eta_propeller = specs.get("propeller_efficiency", 0.85)
        self.eta_motor = specs.get("motor_efficiency", 0.95)

        # ISA Atmosphere Model
        self.atmosphere = ISAAtmosphere()

    def get_current_mass(self, fuel_remaining_kg: float, battery_mass_kg: float = None) -> float:
        """Calculates instantaneous aircraft mass based on remaining fuel and battery mass.

        aircraft_mass_kg = dry_mass_kg + payload_mass_kg + battery_mass_kg + remaining_fuel_kg
        """
        fuel = max(0.0, float(fuel_remaining_kg))
        bat = float(battery_mass_kg) if battery_mass_kg is not None else self.battery_weight
        return self.empty_weight + self.payload + bat + fuel

    def calculate_aerodynamics(
        self,
        altitude_m: float,
        speed_kmph: float,
        mass_kg: float,
        climb_rate_ms: float = 0.0
    ) -> dict:
        """Calculates aerodynamic forces, coefficients, and required power domains.

        Returns:
            dict: Containing density, CL, CDi, CD, Drag, P_prop, P_shaft, P_elec
        """
        atmos = self.atmosphere.get_conditions(altitude_m)
        rho = atmos["density_kg_m3"]

        v_ms = max(1.0, float(speed_kmph) / 3.6)
        mass = max(100.0, float(mass_kg))

        # Weight & Lift (L = W for level flight)
        weight_n = mass * self.G
        lift_n = weight_n

        # Dynamic pressure q = 0.5 * rho * v^2
        q = 0.5 * rho * (v_ms ** 2)

        # Lift Coefficient CL
        cl = lift_n / (q * self.wing_area)
        cl_clamped = min(cl, self.cl_max)

        # Induced Drag Coefficient CDi = CL^2 / (pi * AR * e)
        cdi = (cl_clamped ** 2) / (math.pi * self.aspect_ratio * self.oswald_eff)

        # Total Drag Coefficient CD = CD0 + CDi
        cd = self.cd0 + cdi

        # Drag Force D = q * S * CD
        drag_n = max(0.0, q * self.wing_area * cd)

        # 1. Propulsive Power (Level Flight) = D * V
        p_prop_level_w = drag_n * v_ms

        # 2. Extra Climb Power = m * g * Vz
        p_climb_w = max(0.0, mass * self.G * float(climb_rate_ms))

        # Total Propulsive Power (Watts & kW)
        p_prop_total_w = p_prop_level_w + p_climb_w
        p_prop_kw = p_prop_total_w / 1000.0

        # 3. Mechanical Shaft Power = P_prop / eta_propeller
        p_shaft_kw = p_prop_kw / self.eta_propeller

        # 4. Motor Electrical Input Power = P_shaft / eta_motor
        p_elec_kw = p_shaft_kw / self.eta_motor

        # Enforce minimum idle electrical power floor (10 kW)
        p_elec_kw = max(10.0, p_elec_kw)

        return {
            "altitude_m": altitude_m,
            "density_kg_m3": rho,
            "speed_kmph": speed_kmph,
            "speed_ms": round(v_ms, 2),
            "mass_kg": round(mass, 2),
            "cl": round(cl, 4),
            "cdi": round(cdi, 5),
            "cd0": self.cd0,
            "cd": round(cd, 5),
            "drag_n": round(drag_n, 2),
            "propulsive_power_kw": round(p_prop_kw, 2),
            "shaft_power_kw": round(p_shaft_kw, 2),
            "motor_elec_power_kw": round(p_elec_kw, 2),
            "eta_propeller": self.eta_propeller,
            "eta_motor": self.eta_motor
        }

    def display_information(self):
        print("\n===== Aircraft Specifications =====")
        print(f"Aircraft : {self.name}")
        print(f"MTOW : {self.mtow} kg")
        print(f"Empty Weight : {self.empty_weight} kg")
        print(f"Payload : {self.payload} kg")
        print(f"Fuel : {self.fuel} kg")
        print(f"Battery : {self.battery_capacity} kWh")
        print(f"Cruise Speed : {self.cruise_speed} km/h")
        print(f"Maximum Speed : {self.max_speed} km/h")
        print(f"Service Ceiling : {self.ceiling} m")
        print(f"Range : {self.range} km")
        print(f"Endurance : {self.endurance} hr")
        print(f"Wingspan : {self.wing_span} m")
        print(f"Wing Area : {self.wing_area} sq.m")
        print(f"Aspect Ratio : {self.aspect_ratio:.2f}")
        print(f"Oswald Efficiency : {self.oswald_eff}")
        print(f"Zero-Lift Drag CD0: {self.cd0}")
        print(f"Propeller Eff eta : {self.eta_propeller}")
        print(f"Motor Eff eta     : {self.eta_motor}")
        print("==================================")