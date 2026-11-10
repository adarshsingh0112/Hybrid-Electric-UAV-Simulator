"""
Hybrid-Electric Powertrain Integration Module — MALE UAV Digital Twin
Integrates:
- Mean Value Engine Model (MVEM) Aero Piston Engine (simulation/mvem.py)
- Battery Energy Storage System (BESS) (simulation/battery.py)
- Electric Motor / Generator & Inverter Engineering Model
- Power Management Controller (Deterministic Rule-Based Control)

Modes Supported:
1. ENGINE_ONLY: Internal combustion engine delivers primary propulsion power.
2. ELECTRIC_ONLY: Battery powers electric motor for zero-emission quiet loiter.
3. MOTOR_ASSIST: Engine + Electric Motor operate in parallel for peak sprint/climb power.
4. BATTERY_CHARGING: Engine powers propeller propulsion and drives generator to charge battery.
"""

from dataclasses import dataclass, field, asdict
from enum import Enum
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union, Any

from .flight_profile import FlightState
from .mvem import MeanValueEngineModel, EngineState
from .battery import BatteryModel, BatteryConfig, BatteryState, BatteryTelemetry, BatteryChargeState


class HybridOperatingMode(str, Enum):
    ENGINE_ONLY = "ENGINE_ONLY"
    ELECTRIC_ONLY = "ELECTRIC_ONLY"
    MOTOR_ASSIST = "MOTOR_ASSIST"
    BATTERY_CHARGING = "BATTERY_CHARGING"


class HybridControlMode(str, Enum):
    AUTO = "AUTO"
    MANUAL = "MANUAL"


@dataclass
class HybridPowertrainConfig:
    """
    Configuration specs for UAV Parallel Hybrid-Electric Powertrain.
    """
    powertrain_name: str = "DRDO-MALE-UAV-Parallel-Hybrid-Powertrain"
    max_electric_motor_power_w: float = 40000.0   # Continuous max electric motor power [W] (40 kW)
    max_generator_power_w: float = 30000.0        # Continuous max generator charging power [W] (30 kW)
    motor_efficiency: float = 0.94                # Electric motor mechanical efficiency [0.0 - 1.0]
    generator_efficiency: float = 0.92            # Generator electrical efficiency [0.0 - 1.0]
    inverter_efficiency: float = 0.97             # Power electronics inverter efficiency [0.0 - 1.0]
    electric_machine_thermal_capacity_j_per_k: float = 18000.0 # Heat capacity [J/K]
    electric_machine_cooling_coeff_w_per_k: float = 15.0      # Cooling coefficient [W/K]
    max_electric_machine_temp_c: float = 110.0    # Alarm temperature limit [°C]
    engine_idle_throttle_pct: float = 15.0        # Engine idle throttle [%]
    engine_max_continuous_throttle_pct: float = 95.0 # Max continuous throttle [%]
    auto_mode_soc_low_threshold_pct: float = 20.0 # SoC % trigger for auto battery charging
    auto_mode_soc_high_threshold_pct: float = 80.0# SoC % trigger for electric loiter
    auto_mode_high_power_threshold_w: float = 130000.0 # Propulsion power trigger for motor assist [W]

    @classmethod
    def from_json(cls, json_path: Union[str, Path]) -> "HybridPowertrainConfig":
        """Loads hybrid configuration from a JSON file."""
        path = Path(json_path)
        if not path.exists():
            raise FileNotFoundError(f"Hybrid configuration file not found: {path}")

        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        valid_keys = {f.name for f in cls.__dataclass_fields__.values()}
        filtered_data = {k: v for k, v in data.items() if k in valid_keys}
        return cls(**filtered_data)


@dataclass
class ElectricMachineState:
    """
    Physical and thermal state of the Electric Motor / Generator.
    """
    requested_power_w: float       # Requested mechanical power [W] (+ Motor, - Generator)
    actual_power_w: float          # Actual delivered/absorbed mechanical power [W]
    electrical_power_w: float      # Electrical power drawn (+) or generated (-) [W]
    torque_nm: float               # Shaft torque [Nm] (+ Drive, - Drag)
    rpm: float                     # Shaft speed [RPM]
    efficiency: float              # Net efficiency [0.0 - 1.0]
    power_loss_w: float            # Thermal losses in motor & inverter [W]
    temperature_c: float           # Machine temperature [°C]
    is_motor: bool                 # True if operating as motor, False if generator/idle
    is_thermal_warning: bool       # True if temperature exceeds warning limit


@dataclass
class HybridPowertrainState:
    """
    Internal state representation of the Hybrid Powertrain.
    """
    mode: HybridOperatingMode
    control_mode: HybridControlMode
    propulsion_power_req_w: float
    propulsion_power_actual_w: float
    propulsion_power_deficit_w: float
    engine_power_propulsion_w: float
    engine_power_generator_w: float
    engine_power_total_w: float
    motor_power_propulsion_w: float
    battery_power_w: float
    is_power_constrained: bool


@dataclass
class HybridPowertrainTelemetry:
    """
    Exportable telemetry snapshot for the complete Hybrid Powertrain System.
    Exposes unified propulsion, engine, battery, electric machine, and system state metrics.
    """
    timestamp_s: float
    operating_mode: str               # ENGINE_ONLY, ELECTRIC_ONLY, MOTOR_ASSIST, BATTERY_CHARGING
    control_mode: str                 # AUTO or MANUAL
    propulsion_power_req_w: float     # Total requested propulsion power [W]
    propulsion_power_actual_w: float  # Total delivered propulsion power [W]
    propulsion_power_deficit_w: float # Unmet power demand deficit [W]
    engine_power_contribution_w: float # IC engine mechanical power contribution [W]
    engine_power_contribution_hp: float # IC engine mechanical power contribution [HP]
    motor_power_contribution_w: float  # Electric motor mechanical power contribution [W]
    motor_power_contribution_hp: float  # Electric motor mechanical power contribution [HP]
    engine_rpm: float                 # Engine crankshaft speed [RPM]
    engine_throttle_pct: float        # Engine throttle setting [%]
    engine_fuel_flow_lph: float       # Engine fuel consumption rate [L/h]
    battery_soc_pct: float            # Battery State of Charge [%]
    battery_soh_pct: float            # Battery State of Health [%]
    battery_voltage_v: float          # Battery terminal voltage [V]
    battery_current_a: float          # Battery current [A] (+ Discharge, - Charge)
    battery_power_w: float            # Battery electrical power [W] (+ Discharge, - Charge)
    battery_temperature_c: float      # Battery pack temperature [°C]
    battery_remaining_energy_kwh: float # Battery usable stored energy [kWh]
    battery_is_constrained: bool      # Battery limit flag
    electric_machine_power_w: float   # Electric machine mechanical power [W]
    electric_machine_torque_nm: float # Electric machine shaft torque [Nm]
    electric_machine_efficiency: float# Net electric machine efficiency [0.0 - 1.0]
    electric_machine_temperature_c: float # Electric machine temperature [°C]
    electric_machine_mode: str        # MOTOR, GENERATOR, IDLE
    is_system_power_constrained: bool # System-wide power deficit / boundary flag
    energy_flow_direction: str        # Flow description
    provenance: str = "SIMULATED"

    def to_dict(self) -> Dict[str, Any]:
        """Serializes telemetry object to Python dictionary."""
        return asdict(self)


class ElectricMachineModel:
    """
    Physical engineering model of an Electric Motor / Generator & Inverter unit.
    Converts between electrical energy (DC battery side) and mechanical shaft power.
    """

    def __init__(self, config: HybridPowertrainConfig):
        self.config = config
        self.temperature_c: float = 25.0

    def reset(self, initial_temp_c: float = 25.0):
        self.temperature_c = float(initial_temp_c)

    def step(
        self,
        mech_power_request_w: float,
        shaft_rpm: float,
        ambient_temp_c: float,
        dt_s: float = 0.05
    ) -> ElectricMachineState:
        """
        Integrates electric machine state forward by dt_s.
        :param mech_power_request_w: Requested mechanical power [W] (+ Motor, - Generator)
        :param shaft_rpm: Shaft rotational speed [RPM]
        :param ambient_temp_c: Outside ambient air temperature [°C]
        :param dt_s: Integration time step [seconds]
        """
        dt_s = max(0.001, float(dt_s))
        rpm = max(100.0, float(shaft_rpm))
        omega = (rpm * 2.0 * math.pi) / 60.0 # Angular velocity in rad/s

        p_req = float(mech_power_request_w)

        if abs(p_req) < 1.0:
            # Idle
            p_actual_mech = 0.0
            p_elec = 0.0
            torque_nm = 0.0
            efficiency = 1.0
            p_loss_w = 0.0
            is_motor = True
            mode_str = "IDLE"
        elif p_req > 0.0:
            # Motor Mode (Battery -> Motor -> Mechanical Shaft)
            mode_str = "MOTOR"
            is_motor = True
            p_actual_mech = min(p_req, self.config.max_electric_motor_power_w)
            net_eff = self.config.motor_efficiency * self.config.inverter_efficiency
            efficiency = net_eff
            
            # Electrical power drawn from battery: P_elec = P_mech / net_eff
            p_elec = p_actual_mech / max(0.1, net_eff)
            p_loss_w = p_elec - p_actual_mech
            torque_nm = p_actual_mech / omega if omega > 0 else 0.0
        else:
            # Generator Mode (Mechanical Shaft -> Generator -> Battery)
            mode_str = "GENERATOR"
            is_motor = False
            p_abs_req = abs(p_req)
            p_actual_mech_abs = min(p_abs_req, self.config.max_generator_power_w)
            p_actual_mech = -p_actual_mech_abs
            
            net_eff = self.config.generator_efficiency * self.config.inverter_efficiency
            efficiency = net_eff
            
            # Electrical power generated for battery (negative value): P_elec = - (P_mech * net_eff)
            p_elec = -(p_actual_mech_abs * net_eff)
            p_loss_w = p_actual_mech_abs - abs(p_elec)
            torque_nm = -p_actual_mech_abs / omega if omega > 0 else 0.0

        # Thermal evolution: C_p * dT/dt = P_loss - k_cool * (T - T_amb)
        p_cooling_w = self.config.electric_machine_cooling_coeff_w_per_k * (self.temperature_c - ambient_temp_c)
        net_thermal_power_w = p_loss_w - p_cooling_w
        
        d_temp_dt = net_thermal_power_w / self.config.electric_machine_thermal_capacity_j_per_k
        self.temperature_c += d_temp_dt * dt_s

        is_thermal_warn = self.temperature_c >= self.config.max_electric_machine_temp_c

        return ElectricMachineState(
            requested_power_w=p_req,
            actual_power_w=p_actual_mech,
            electrical_power_w=p_elec,
            torque_nm=torque_nm,
            rpm=rpm,
            efficiency=efficiency,
            power_loss_w=p_loss_w,
            temperature_c=self.temperature_c,
            is_motor=is_motor,
            is_thermal_warning=is_thermal_warn
        )


class PowerManagementController:
    """
    Deterministic rule-based supervisor that determines the active HybridOperatingMode.
    Decoupled from physical equations to allow future optimization or AI control replacement.
    """

    def __init__(self, config: HybridPowertrainConfig):
        self.config = config

    def select_operating_mode(
        self,
        propulsion_power_req_w: float,
        battery_soc_pct: float,
        engine_max_power_w: float,
        battery_is_constrained: bool,
        manual_override_mode: Optional[HybridOperatingMode] = None
    ) -> HybridOperatingMode:
        """
        Determines operating mode based on propulsion demand, battery SoC, and power constraints.
        """
        # 1. Respect manual override if specified
        if manual_override_mode is not None:
            return manual_override_mode

        # 2. Deterministic Rule-Based Control (AUTO Mode)
        p_req = max(0.0, float(propulsion_power_req_w))

        # Rule A: Critically Low Battery SoC (< 20%) -> Force Battery Charging (or Engine Only if at max load)
        if battery_soc_pct < self.config.auto_mode_soc_low_threshold_pct:
            if p_req > (engine_max_power_w * 0.90):
                return HybridOperatingMode.ENGINE_ONLY
            return HybridOperatingMode.BATTERY_CHARGING

        # Rule B: High Propulsion Power Demand (> 130 kW or > 85% Engine Max) & Sufficient Battery -> Motor Assist
        if (p_req > self.config.auto_mode_high_power_threshold_w or p_req > (engine_max_power_w * 0.85)) and battery_soc_pct > 20.0:
            return HybridOperatingMode.MOTOR_ASSIST

        # Rule C: Low/Loiter Propulsion Power Demand (< 35 kW) & High Battery SoC (> 80%) -> Pure Electric Loiter
        if p_req < (self.config.max_electric_motor_power_w * 0.85) and battery_soc_pct >= self.config.auto_mode_soc_high_threshold_pct:
            return HybridOperatingMode.ELECTRIC_ONLY

        # Rule D: Nominal Default -> Engine Only
        return HybridOperatingMode.ENGINE_ONLY


class HybridPowertrain:
    """
    Combined Hybrid-Electric Powertrain Simulator.
    Coordinates the Mean Value Engine Model (MVEM), Battery Energy Storage System (BESS),
    Electric Machine, and Power Management Controller.
    """

    def __init__(
        self,
        config: Optional[Union[HybridPowertrainConfig, str, Path, Dict[str, Any]]] = None,
        engine: Optional[MeanValueEngineModel] = None,
        battery: Optional[BatteryModel] = None
    ):
        # Resolve config
        if config is None:
            default_json = Path(__file__).resolve().parent.parent / "configs" / "hybrid_config.json"
            if default_json.exists():
                self.config = HybridPowertrainConfig.from_json(default_json)
            else:
                self.config = HybridPowertrainConfig()
        elif isinstance(config, (str, Path)):
            self.config = HybridPowertrainConfig.from_json(config)
        elif isinstance(config, dict):
            self.config = HybridPowertrainConfig(**config)
        elif isinstance(config, HybridPowertrainConfig):
            self.config = config
        else:
            self.config = HybridPowertrainConfig()

        # Instantiate sub-models
        self.engine = engine if engine is not None else MeanValueEngineModel()
        self.battery = battery if battery is not None else BatteryModel()
        self.electric_machine = ElectricMachineModel(self.config)
        self.controller = PowerManagementController(self.config)

        self.sim_time_s: float = 0.0

    def reset(self):
        """Resets all powertrain components to healthy baseline state."""
        self.sim_time_s = 0.0
        self.engine.reset(idle=False)
        self.battery.reset()
        self.electric_machine.reset()

    def update(
        self,
        propulsion_power_request_w: float,
        flight_state: FlightState,
        ambient_temp_c: float,
        dt_s: float = 0.05,
        mode_override: Optional[HybridOperatingMode] = None
    ) -> HybridPowertrainTelemetry:
        """
        Integrates hybrid-electric powertrain forward by dt_s.

        :param propulsion_power_request_w: Total requested propulsion power [W]
        :param flight_state: Flight environment state (altitude, airspeed, ISA conditions)
        :param ambient_temp_c: Outside ambient air temperature [°C]
        :param dt_s: Integration step size [seconds]
        :param mode_override: Manual mode override (ENGINE_ONLY, ELECTRIC_ONLY, MOTOR_ASSIST, BATTERY_CHARGING)
        """
        dt_s = max(0.001, float(dt_s))
        self.sim_time_s += dt_s
        p_prop_req = max(0.0, float(propulsion_power_request_w))
        alt_ft = flight_state.altitude_ft

        # 1. Determine Engine Available Mechanical Power at Current Altitude
        eng_max_hp = self.engine.get_max_available_power_hp(alt_ft)
        eng_max_w = eng_max_hp * 745.7

        # 2. Controller Selects Operating Mode
        control_mode = HybridControlMode.MANUAL if mode_override is not None else HybridControlMode.AUTO
        op_mode = self.controller.select_operating_mode(
            propulsion_power_req_w=p_prop_req,
            battery_soc_pct=self.battery.soc_pct,
            engine_max_power_w=eng_max_w,
            battery_is_constrained=False,
            manual_override_mode=mode_override
        )

        # Variables to populate per mode
        p_eng_prop_w = 0.0
        p_eng_gen_w = 0.0
        p_eng_total_req_w = 0.0
        p_em_req_mech_w = 0.0
        target_engine_throttle_pct = 0.0
        flow_direction = "IDLE"

        # 3. Process Power Split Based on Active Operating Mode
        if op_mode == HybridOperatingMode.ENGINE_ONLY:
            # Engine supplies 100% of propulsion power. Motor and Battery idle.
            flow_direction = "ENGINE_TO_PROPULSION"
            p_eng_total_req_w = p_prop_req
            p_eng_prop_w = min(p_prop_req, eng_max_w)
            
            target_engine_throttle_pct = min(
                self.config.engine_max_continuous_throttle_pct,
                max(self.config.engine_idle_throttle_pct, (p_eng_total_req_w / max(1.0, eng_max_w)) * 100.0)
            )
            p_em_req_mech_w = 0.0

        elif op_mode == HybridOperatingMode.ELECTRIC_ONLY:
            # Battery powers electric motor. Engine runs at idle throttle.
            flow_direction = "BATTERY_TO_PROPULSION"
            target_engine_throttle_pct = self.config.engine_idle_throttle_pct
            p_eng_prop_w = 0.0 # Engine disengaged from propeller load
            
            p_em_req_mech_w = min(p_prop_req, self.config.max_electric_motor_power_w)

        elif op_mode == HybridOperatingMode.MOTOR_ASSIST:
            # Engine operates at high throttle; Motor assists with remaining demand.
            flow_direction = "DUAL_TO_PROPULSION"
            target_engine_throttle_pct = self.config.engine_max_continuous_throttle_pct
            p_eng_prop_w = eng_max_w # Engine at max continuous output
            
            p_remaining_req = max(0.0, p_prop_req - p_eng_prop_w)
            p_em_req_mech_w = min(p_remaining_req, self.config.max_electric_motor_power_w)

        elif op_mode == HybridOperatingMode.BATTERY_CHARGING:
            # Engine powers propeller AND drives generator to charge battery.
            flow_direction = "ENGINE_TO_BATTERY"
            p_eng_prop_w = min(p_prop_req, eng_max_w)
            
            # Additional generator mechanical power request
            p_gen_mech_target = self.config.max_generator_power_w
            
            # Engine total demand (Propulsion has priority over charging)
            p_eng_total_req_w = p_eng_prop_w + p_gen_mech_target
            if p_eng_total_req_w > eng_max_w:
                p_eng_gen_w = max(0.0, eng_max_w - p_eng_prop_w)
                p_eng_total_req_w = eng_max_w
            else:
                p_eng_gen_w = p_gen_mech_target

            target_engine_throttle_pct = min(
                100.0,
                max(self.config.engine_idle_throttle_pct, (p_eng_total_req_w / max(1.0, eng_max_w)) * 100.0)
            )
            p_em_req_mech_w = -p_eng_gen_w # Negative mechanical power for generator

        # 4. Integrate Engine Model with Modulated Throttle
        # Create modified flight state with target throttle
        mod_flight = FlightState(
            time_s=flight_state.time_s,
            altitude_ft=flight_state.altitude_ft,
            altitude_m=flight_state.altitude_m,
            throttle_pct=target_engine_throttle_pct,
            airspeed_mps=flight_state.airspeed_mps,
            ambient_temp_c=flight_state.ambient_temp_c,
            ambient_pressure_bar=flight_state.ambient_pressure_bar,
            air_density_kgpm3=flight_state.air_density_kgpm3,
            phase=flight_state.phase
        )

        engine_state: EngineState = self.engine.step(mod_flight, dt_s=dt_s)
        actual_eng_power_w = engine_state.power_hp * 745.7

        # In ENGINE_ONLY or BATTERY_CHARGING, actual engine propulsion contribution is bounded by engine simulation output
        if op_mode == HybridOperatingMode.ENGINE_ONLY:
            actual_eng_prop_w = actual_eng_power_w
        elif op_mode == HybridOperatingMode.MOTOR_ASSIST:
            actual_eng_prop_w = actual_eng_power_w
        elif op_mode == HybridOperatingMode.BATTERY_CHARGING:
            actual_eng_prop_w = min(p_prop_req, actual_eng_power_w)
            actual_eng_gen_mech_w = max(0.0, actual_eng_power_w - actual_eng_prop_w)
            p_em_req_mech_w = -actual_eng_gen_mech_w
        else: # ELECTRIC_ONLY
            actual_eng_prop_w = 0.0

        # 5. Integrate Electric Machine Model
        em_state: ElectricMachineState = self.electric_machine.step(
            mech_power_request_w=p_em_req_mech_w,
            shaft_rpm=engine_state.rpm,
            ambient_temp_c=ambient_temp_c,
            dt_s=dt_s
        )

        # 6. Integrate Battery Model with Electrical Power Demand (P_elec_in)
        # em_state.electrical_power_w is (+) for motor discharge, (-) for generator charge
        batt_telemetry: BatteryTelemetry = self.battery.update(
            power_request=em_state.electrical_power_w,
            ambient_temperature=ambient_temp_c,
            dt=dt_s
        )

        # If battery power was constrained, re-adjust actual motor mechanical output
        if op_mode == HybridOperatingMode.ELECTRIC_ONLY or op_mode == HybridOperatingMode.MOTOR_ASSIST:
            if batt_telemetry.is_power_constrained and batt_telemetry.power_actual_w < em_state.electrical_power_w:
                actual_motor_mech_w = batt_telemetry.power_actual_w * self.config.motor_efficiency * self.config.inverter_efficiency
            else:
                actual_motor_mech_w = em_state.actual_power_w
        else:
            actual_motor_mech_w = 0.0

        # 7. Total Delivered Propulsion Power & Power Deficit
        p_prop_actual_w = actual_eng_prop_w + actual_motor_mech_w
        p_deficit_w = max(0.0, p_prop_req - p_prop_actual_w)

        is_system_constrained = (
            p_deficit_w > 10.0 or
            batt_telemetry.is_power_constrained or
            em_state.is_thermal_warning
        )

        return HybridPowertrainTelemetry(
            timestamp_s=round(self.sim_time_s, 2),
            operating_mode=op_mode.value,
            control_mode=control_mode.value,
            propulsion_power_req_w=round(p_prop_req, 1),
            propulsion_power_actual_w=round(p_prop_actual_w, 1),
            propulsion_power_deficit_w=round(p_deficit_w, 1),
            engine_power_contribution_w=round(actual_eng_prop_w, 1),
            engine_power_contribution_hp=round(actual_eng_prop_w / 745.7, 1),
            motor_power_contribution_w=round(actual_motor_mech_w, 1),
            motor_power_contribution_hp=round(actual_motor_mech_w / 745.7, 1),
            engine_rpm=round(engine_state.rpm, 1),
            engine_throttle_pct=round(target_engine_throttle_pct, 1),
            engine_fuel_flow_lph=round(engine_state.fuel_flow_lph, 2),
            battery_soc_pct=round(batt_telemetry.soc_pct, 2),
            battery_soh_pct=round(batt_telemetry.soh_pct, 2),
            battery_voltage_v=round(batt_telemetry.terminal_voltage_v, 2),
            battery_current_a=round(batt_telemetry.current_a, 2),
            battery_power_w=round(batt_telemetry.power_actual_w, 1),
            battery_temperature_c=round(batt_telemetry.temperature_c, 2),
            battery_remaining_energy_kwh=round(batt_telemetry.usable_energy_kwh, 3),
            battery_is_constrained=batt_telemetry.is_power_constrained,
            electric_machine_power_w=round(em_state.actual_power_w, 1),
            electric_machine_torque_nm=round(em_state.torque_nm, 1),
            electric_machine_efficiency=round(em_state.efficiency, 3),
            electric_machine_temperature_c=round(em_state.temperature_c, 2),
            electric_machine_mode="MOTOR" if em_state.is_motor and abs(em_state.actual_power_w) > 1 else ("GENERATOR" if not em_state.is_motor else "IDLE"),
            is_system_power_constrained=is_system_constrained,
            energy_flow_direction=flow_direction,
            provenance="SIMULATED"
        )
