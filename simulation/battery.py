"""
Battery Energy Storage System (BESS) — Physics-Based Simulation Module
Designed for Hybrid-Electric MALE UAV Power Plant Applications.

Physics & Engineering Model:
1. State of Charge (SoC %) & State of Health (SoH %) Tracking
2. Non-linear Open Circuit Voltage (OCV = f(SoC)) & Terminal Voltage Under Load
3. Equivalent Circuit Internal Resistance (R_internal = f(SoH, T_batt)) & Joule Losses (I^2 * R)
4. Thermal Heat Generation & Lumped Convective Cooling Dynamics (dT/dt)
5. Current & Power Boundary Constraints (Voltage limits, C-rate limits, SoC cut-offs)
"""

from dataclasses import dataclass, field, asdict
from enum import Enum
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union, Any


class BatteryChargeState(str, Enum):
    IDLE = "IDLE"
    DISCHARGING = "DISCHARGING"
    CHARGING = "CHARGING"
    FULL = "FULL"
    DEPLETED = "DEPLETED"


@dataclass
class BatteryConfig:
    """
    Configuration parameters for UAV Battery Energy Storage System (BESS).
    Includes physical electrical specs, thermal constants, and boundary limits.
    """
    battery_name: str = "DRDO-MALE-UAV-HV-Battery-18.5kWh"
    nominal_capacity_ah: float = 50.0        # Nominal cell pack capacity [Ah]
    nominal_voltage_v: float = 370.0          # Nominal pack voltage [V]
    min_voltage_v: float = 300.0              # Cut-off minimum discharge voltage [V]
    max_voltage_v: float = 420.0              # Fully charged maximum voltage [V]
    nominal_internal_resistance_ohm: float = 0.08 # Pack internal resistance [Ohm]
    max_discharge_current_a: float = 200.0    # Continuous max discharge current [A]
    max_charge_current_a: float = 100.0       # Continuous max charge current [A]
    max_discharge_power_w: float = 65000.0    # Max continuous discharge power [W]
    max_charge_power_w: float = 35000.0       # Max continuous charge power [W]
    charge_efficiency: float = 0.96           # Coulombic charge efficiency [0.0 - 1.0]
    discharge_efficiency: float = 0.98        # Discharge efficiency factor [0.0 - 1.0]
    thermal_capacity_j_per_k: float = 45000.0 # Pack lumped heat capacity [J/K]
    cooling_coefficient_w_per_k: float = 25.0 # Convective cooling rate h*A [W/K]
    aging_resistance_factor: float = 0.5      # Resistance growth factor per (1 - SoH/100)
    initial_soc_pct: float = 100.0            # Initial State of Charge [0.0 - 100.0%]
    initial_soh_pct: float = 100.0            # Initial State of Health [0.0 - 100.0%]
    initial_temp_c: float = 25.0              # Initial pack temperature [°C]
    ocv_soc_curve: List[Tuple[float, float]] = field(default_factory=lambda: [
        (0.0, 300.0),
        (10.0, 335.0),
        (20.0, 350.0),
        (50.0, 370.0),
        (80.0, 390.0),
        (90.0, 405.0),
        (100.0, 420.0)
    ])

    @classmethod
    def from_json(cls, json_path: Union[str, Path]) -> "BatteryConfig":
        """Loads battery configuration from a JSON file."""
        path = Path(json_path)
        if not path.exists():
            raise FileNotFoundError(f"Battery configuration file not found: {path}")
        
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        
        # Convert OCV list of dicts to list of tuples if needed
        if "ocv_soc_curve" in data and isinstance(data["ocv_soc_curve"], list):
            curve = []
            for item in data["ocv_soc_curve"]:
                if isinstance(item, dict):
                    curve.append((float(item["soc_pct"]), float(item["ocv_v"])))
                elif isinstance(item, (list, tuple)):
                    curve.append((float(item[0]), float(item[1])))
            data["ocv_soc_curve"] = curve

        # Filter out unknown keys for clean initialization
        valid_keys = {f.name for f in cls.__dataclass_fields__.values()}
        filtered_data = {k: v for k, v in data.items() if k in valid_keys}
        return cls(**filtered_data)


@dataclass
class BatteryState:
    """
    Internal state representation of the battery simulation.
    """
    soc_pct: float             # State of Charge [0.0 - 100.0%]
    soh_pct: float             # State of Health [0.0 - 100.0%]
    temperature_c: float       # Pack temperature [°C]
    usable_capacity_ah: float  # Actual usable capacity scaled by SoH [Ah]
    stored_energy_wh: float    # Usable energy remaining [Wh]
    max_usable_energy_wh: float# Total energy capacity at current SoH [Wh]


@dataclass
class BatteryTelemetry:
    """
    Exportable telemetry snapshot emitted at each simulation step.
    Contains complete electric, thermal, energy, and diagnostic parameters.
    """
    timestamp_s: float
    soc_pct: float                    # State of Charge [0.0 - 100.0%]
    soh_pct: float                    # State of Health [0.0 - 100.0%]
    terminal_voltage_v: float         # Battery terminal voltage [V]
    open_circuit_voltage_v: float     # Open Circuit Voltage OCV [V]
    internal_resistance_ohm: float    # Internal resistance [Ohm]
    current_a: float                  # Current [A] (Positive = Discharge, Negative = Charge)
    power_demand_w: float             # Requested power demand [W]
    power_actual_w: float             # Actual delivered/absorbed power [W]
    power_loss_w: float               # Resistive internal power loss I^2 * R [W]
    usable_energy_wh: float           # Stored energy remaining [Wh]
    usable_energy_kwh: float          # Stored energy remaining [kWh]
    temperature_c: float              # Pack temperature [°C]
    charge_state: str                 # IDLE, DISCHARGING, CHARGING, FULL, DEPLETED
    is_power_constrained: bool        # True if demand was limited by hardware/SoC boundaries
    provenance: str = "SIMULATED"

    def to_dict(self) -> Dict[str, Any]:
        """Serializes telemetry object to dictionary."""
        return asdict(self)


class BatteryModel:
    """
    Physically interpretable Battery Energy Storage System (BESS) simulator.
    Evaluates voltage dynamics, Joule heating, thermal cooling, SoC evolution,
    and SoH degradation constraints.
    """

    def __init__(
        self,
        config: Optional[Union[BatteryConfig, str, Path, Dict[str, Any]]] = None,
        initial_soc_pct: Optional[float] = None,
        initial_soh_pct: Optional[float] = None,
        initial_temp_c: Optional[float] = None
    ):
        # Resolve configuration object
        if config is None:
            # Check default path configs/battery_config.json
            default_json = Path(__file__).resolve().parent.parent / "configs" / "battery_config.json"
            if default_json.exists():
                self.config = BatteryConfig.from_json(default_json)
            else:
                self.config = BatteryConfig()
        elif isinstance(config, (str, Path)):
            self.config = BatteryConfig.from_json(config)
        elif isinstance(config, dict):
            self.config = BatteryConfig(**config)
        elif isinstance(config, BatteryConfig):
            self.config = config
        else:
            self.config = BatteryConfig()

        self.sim_time_s: float = 0.0
        
        # Override initial conditions if specified
        soc_start = initial_soc_pct if initial_soc_pct is not None else self.config.initial_soc_pct
        soh_start = initial_soh_pct if initial_soh_pct is not None else self.config.initial_soh_pct
        temp_start = initial_temp_c if initial_temp_c is not None else self.config.initial_temp_c

        self.reset(soc_pct=soc_start, soh_pct=soh_start, temp_c=temp_start)

    def reset(
        self,
        soc_pct: Optional[float] = None,
        soh_pct: Optional[float] = None,
        temp_c: Optional[float] = None
    ):
        """Resets battery simulator to specified or baseline initial conditions."""
        self.sim_time_s = 0.0
        
        soc = soc_pct if soc_pct is not None else self.config.initial_soc_pct
        soh = soh_pct if soh_pct is not None else self.config.initial_soh_pct
        temp = temp_c if temp_c is not None else self.config.initial_temp_c

        self.soc_pct = max(0.0, min(100.0, float(soc)))
        self.soh_pct = max(0.0, min(100.0, float(soh)))
        self.temperature_c = float(temp)

    def get_open_circuit_voltage(self, soc_pct: float) -> float:
        """
        Calculates Open Circuit Voltage (OCV) using piecewise linear interpolation
        over the configured OCV vs SoC curve.
        """
        soc = max(0.0, min(100.0, soc_pct))
        curve = sorted(self.config.ocv_soc_curve, key=lambda x: x[0])

        if soc <= curve[0][0]:
            return curve[0][1]
        if soc >= curve[-1][0]:
            return curve[-1][1]

        for i in range(len(curve) - 1):
            s0, v0 = curve[i]
            s1, v1 = curve[i + 1]
            if s0 <= soc <= s1:
                frac = (soc - s0) / (s1 - s0) if s1 > s0 else 0.0
                return v0 + frac * (v1 - v0)

        return self.config.nominal_voltage_v

    def get_effective_internal_resistance(self, soh_pct: float, temp_c: float) -> float:
        """
        Calculates effective internal resistance considering SoH degradation and thermal effects.
        R_internal = R_nom * (1 + aging_factor * (1 - SoH/100)) * exp(0.012 * (25 - T_batt))
        """
        soh_norm = max(0.01, min(1.0, soh_pct / 100.0))
        r_aging = self.config.nominal_internal_resistance_ohm * (
            1.0 + self.config.aging_resistance_factor * (1.0 - soh_norm)
        )
        # Cold temperature increases electrolyte viscosity & internal resistance
        r_thermal_factor = math.exp(0.012 * (25.0 - temp_c))
        r_thermal_factor = max(0.6, min(3.0, r_thermal_factor))
        return r_aging * r_thermal_factor

    def step(
        self,
        power_request_w: float,
        ambient_temp_c: float,
        dt_s: float = 0.05
    ) -> BatteryTelemetry:
        """
        Integrates battery state forward by dt_s given requested electrical power.

        :param power_request_w: Requested power [W] (Positive = Discharge, Negative = Charge)
        :param ambient_temp_c: Outside ambient air temperature [°C]
        :param dt_s: Integration time step [seconds]
        """
        dt_s = max(0.001, float(dt_s))
        self.sim_time_s += dt_s

        # 1. State of Health (SoH) scaling on usable capacity
        usable_capacity_ah = self.config.nominal_capacity_ah * (self.soh_pct / 100.0)

        # 2. Open Circuit Voltage & Internal Resistance
        ocv_v = self.get_open_circuit_voltage(self.soc_pct)
        r_int = self.get_effective_internal_resistance(self.soh_pct, self.temperature_c)

        # 3. Determine Constraints and Solve Current I
        is_constrained = False
        target_p_w = float(power_request_w)

        # Handle SoC boundary cut-offs
        if self.soc_pct <= 0.5 and target_p_w > 0.0:
            # Battery depleted: cannot deliver discharge power
            target_p_w = 0.0
            is_constrained = True
        elif self.soc_pct >= 99.8 and target_p_w < 0.0:
            # Battery fully charged: cannot accept charge power
            target_p_w = 0.0
            is_constrained = True

        # Enforce configurable power limits
        if target_p_w > self.config.max_discharge_power_w:
            target_p_w = self.config.max_discharge_power_w
            is_constrained = True
        elif target_p_w < -self.config.max_charge_power_w:
            target_p_w = -self.config.max_charge_power_w
            is_constrained = True

        # Solve for current I from quadratic equation: R_int * I^2 - OCV * I + P_actual = 0
        # P_actual = V_term * I = (OCV - I * R_int) * I
        if abs(target_p_w) < 1e-4:
            # Idle
            i_a = 0.0
            p_actual_w = 0.0
            v_term_v = ocv_v
        elif target_p_w > 0.0:
            # Discharge (I > 0)
            # Theoretical max power limit before voltage collapse (Discriminant = 0)
            p_max_theoretical = (ocv_v ** 2) / (4.0 * r_int)
            if target_p_w > p_max_theoretical:
                p_discharge_req = p_max_theoretical * 0.98
                is_constrained = True
            else:
                p_discharge_req = target_p_w

            discriminant = (ocv_v ** 2) - (4.0 * r_int * p_discharge_req)
            if discriminant < 0.0:
                p_discharge_req = (ocv_v ** 2) / (4.0 * r_int) * 0.98
                discriminant = max(0.0, (ocv_v ** 2) - (4.0 * r_int * p_discharge_req))
                is_constrained = True

            i_a = (ocv_v - math.sqrt(discriminant)) / (2.0 * r_int)

            # Check continuous max discharge current limit
            if i_a > self.config.max_discharge_current_a:
                i_a = self.config.max_discharge_current_a
                is_constrained = True

            v_term_v = ocv_v - i_a * r_int

            # Check cut-off minimum terminal voltage limit
            if v_term_v < self.config.min_voltage_v:
                v_term_v = self.config.min_voltage_v
                i_a = max(0.0, (ocv_v - v_term_v) / r_int)
                is_constrained = True

            p_actual_w = v_term_v * i_a

        else:
            # Charge (I < 0, target_p_w < 0)
            p_charge_req = target_p_w
            # Quadratic: R_int * I^2 - OCV * I + P = 0
            # For P < 0, discriminant is always positive: OCV^2 - 4 * R_int * P > 0
            discriminant = (ocv_v ** 2) - (4.0 * r_int * p_charge_req)
            i_a = (ocv_v - math.sqrt(discriminant)) / (2.0 * r_int) # Negative value for I

            # Check continuous max charge current limit
            if abs(i_a) > self.config.max_charge_current_a:
                i_a = -self.config.max_charge_current_a
                is_constrained = True

            v_term_v = ocv_v - i_a * r_int # v_term_v > ocv_v when i_a < 0

            # Check maximum terminal voltage limit
            if v_term_v > self.config.max_voltage_v:
                v_term_v = self.config.max_voltage_v
                i_a = -(v_term_v - ocv_v) / r_int
                is_constrained = True

            p_actual_w = v_term_v * i_a # Negative value

        # 4. Internal Resistive Heat Loss P_loss = I^2 * R_int
        p_loss_w = (i_a ** 2) * r_int

        # 5. State of Charge (SoC) Integration
        # Coulombic throughput delta Ah
        if i_a >= 0.0:
            # Discharge
            delta_ah = (i_a * dt_s) / 3600.0
        else:
            # Charge with efficiency
            delta_ah = (i_a * self.config.charge_efficiency * dt_s) / 3600.0

        if usable_capacity_ah > 0.0:
            delta_soc = -(delta_ah / usable_capacity_ah) * 100.0
        else:
            delta_soc = 0.0

        self.soc_pct = max(0.0, min(100.0, self.soc_pct + delta_soc))

        # 6. Thermal Evolution Integration (Joule Heating + Convective Cooling)
        # m*C_p * dT/dt = P_loss - h*A * (T_batt - T_amb)
        p_cooling_w = self.config.cooling_coefficient_w_per_k * (self.temperature_c - ambient_temp_c)
        net_thermal_power_w = p_loss_w - p_cooling_w
        
        d_temp_dt = net_thermal_power_w / self.config.thermal_capacity_j_per_k
        self.temperature_c += d_temp_dt * dt_s

        # 7. Energy Metrics & Charge State Determination
        usable_energy_wh = (self.soc_pct / 100.0) * usable_capacity_ah * ocv_v
        usable_energy_kwh = usable_energy_wh / 1000.0

        if self.soc_pct <= 0.1:
            state_enum = BatteryChargeState.DEPLETED
        elif self.soc_pct >= 99.9:
            state_enum = BatteryChargeState.FULL
        elif i_a > 0.5:
            state_enum = BatteryChargeState.DISCHARGING
        elif i_a < -0.5:
            state_enum = BatteryChargeState.CHARGING
        else:
            state_enum = BatteryChargeState.IDLE

        return BatteryTelemetry(
            timestamp_s=round(self.sim_time_s, 2),
            soc_pct=round(self.soc_pct, 4),
            soh_pct=round(self.soh_pct, 2),
            terminal_voltage_v=round(v_term_v, 2),
            open_circuit_voltage_v=round(ocv_v, 2),
            internal_resistance_ohm=round(r_int, 4),
            current_a=round(i_a, 2),
            power_demand_w=round(power_request_w, 1),
            power_actual_w=round(p_actual_w, 1),
            power_loss_w=round(p_loss_w, 2),
            usable_energy_wh=round(usable_energy_wh, 2),
            usable_energy_kwh=round(usable_energy_kwh, 4),
            temperature_c=round(self.temperature_c, 2),
            charge_state=state_enum.value,
            is_power_constrained=is_constrained,
            provenance="SIMULATED"
        )

    def update(
        self,
        power_request: float,
        ambient_temperature: float,
        dt: float = 0.05
    ) -> BatteryTelemetry:
        """
        Alias update method adhering to standard digital twin update interface.
        """
        return self.step(
            power_request_w=power_request,
            ambient_temp_c=ambient_temperature,
            dt_s=dt
        )
