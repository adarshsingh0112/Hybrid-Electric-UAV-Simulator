import math


class ISAAtmosphere:
    """International Standard Atmosphere (ISA) Model for UAV Flight Simulation.
    Supports Troposphere (0 to 11,000m) using standard thermodynamic equations.
    """

    # Physical constants
    T0 = 288.15        # Sea level standard temperature (K)
    P0 = 101325.0      # Sea level standard pressure (Pa)
    RHO0 = 1.225       # Sea level standard density (kg/m^3)
    L = 0.0065         # Temperature lapse rate (K/m)
    G0 = 9.80665       # Gravitational acceleration (m/s^2)
    R = 287.0528       # Specific gas constant for dry air (J/(kg*K))
    GAMMA = 1.4        # Ratio of specific heats for air

    def __init__(self):
        self.exponent = self.G0 / (self.R * self.L)  # ~5.25588

    def get_conditions(self, altitude_m: float) -> dict:
        """Calculates atmospheric state at a given altitude in meters.

        Args:
            altitude_m: Geometric altitude above mean sea level (m). Clamped [0, 11000].

        Returns:
            dict: {
                "temperature_k": Temperature in Kelvin,
                "temperature_c": Temperature in Celsius,
                "pressure_pa": Pressure in Pascals,
                "density_kg_m3": Air density in kg/m^3,
                "speed_of_sound_ms": Speed of sound in m/s
            }
        """
        alt = max(0.0, min(11000.0, float(altitude_m)))

        # Temperature at altitude
        temp_k = self.T0 - (self.L * alt)
        temp_c = temp_k - 273.15

        # Pressure at altitude
        pressure_pa = self.P0 * ((temp_k / self.T0) ** self.exponent)

        # Air density at altitude
        density_kg_m3 = pressure_pa / (self.R * temp_k)

        # Speed of sound
        speed_of_sound_ms = math.sqrt(self.GAMMA * self.R * temp_k)

        return {
            "temperature_k": round(temp_k, 2),
            "temperature_c": round(temp_c, 2),
            "pressure_pa": round(pressure_pa, 2),
            "density_kg_m3": round(density_kg_m3, 5),
            "speed_of_sound_ms": round(speed_of_sound_ms, 2)
        }
