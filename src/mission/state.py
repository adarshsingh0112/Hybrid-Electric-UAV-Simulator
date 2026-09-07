from dataclasses import dataclass


@dataclass
class SimulationState:
    """Represents the instantaneous physical state of the UAV during continuous simulation."""

    time_s: float
    mission_phase: str
    altitude_m: float
    target_altitude_m: float
    airspeed_mps: float
    target_airspeed_mps: float
    air_density_kg_m3: float
    aircraft_mass_kg: float
    remaining_fuel_kg: float
    battery_soc_percent: float
    required_propulsive_power_kw: float
    required_motor_elec_power_kw: float
    engine_mechanical_power_kw: float
    generator_electrical_power_kw: float
    battery_power_kw: float
    motor_elec_power_kw: float
    motor_shaft_power_kw: float
    power_deficit_kw: float
    is_power_limited: bool
    motor_rpm: int = 0

    def to_dict(self) -> dict:
        """Converts state into telemetry record dictionary."""
        return {
            "Time (s)": round(self.time_s, 2),
            "Mission Time (min)": round(self.time_s / 60.0, 2),
            "Phase": self.mission_phase,
            "Altitude (m)": round(self.altitude_m, 2),
            "Target Altitude (m)": round(self.target_altitude_m, 2),
            "Airspeed (m/s)": round(self.airspeed_mps, 2),
            "Airspeed (km/h)": round(self.airspeed_mps * 3.6, 2),
            "Speed (km/h)": round(self.airspeed_mps * 3.6, 2),
            "Target Speed (km/h)": round(self.target_airspeed_mps * 3.6, 2),
            "Air Density (kg/m^3)": round(self.air_density_kg_m3, 5),
            "Aircraft Mass (kg)": round(self.aircraft_mass_kg, 2),
            "Fuel Remaining (kg)": round(self.remaining_fuel_kg, 2),
            "Battery SoC (%)": round(self.battery_soc_percent, 2),
            "Propulsion Power (kW)": round(self.required_propulsive_power_kw, 2),
            "Engine Power (kW)": round(self.engine_mechanical_power_kw, 2),
            "Battery Power (kW)": round(self.battery_power_kw, 2),
            "Required Elec Power (kW)": round(self.required_motor_elec_power_kw, 2),
            "Generator Power (kW)": round(self.generator_electrical_power_kw, 2),
            "Motor Elec Power (kW)": round(self.motor_elec_power_kw, 2),
            "Motor Shaft Power (kW)": round(self.motor_shaft_power_kw, 2),
            "Motor RPM": self.motor_rpm,
            "Power Deficit (kW)": round(self.power_deficit_kw, 2),
            "Is Power Limited": self.is_power_limited,
        }
