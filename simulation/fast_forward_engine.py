"""
Fast-Forward Simulation Engine Module (Step 9)
Evaluates future UAV mission scenarios faster than real time using isolated physical models.

Key Principles:
1. Complete Isolation: Simulations execute on distinct model instances without mutating live digital twin state.
2. Physics-Grounded: Reuses exact BESS equivalent circuit, aerodynamic shaft equations, and engine fuel/thermal laws.
3. Wind Triangle Vectorization: Resolves headwind/crosswind on route segments to simulate realistic ground speed and ETAs.
4. Comprehensive Threshold Tracking: Captures first-breach timestamps for fuel reserve, battery reserve, and thermal limits.
5. Deterministic Reproducibility: Returns identical trajectories when supplied with fixed scenario seeds.
"""

from dataclasses import dataclass, field, asdict
from enum import Enum
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Union

from mission.mission_state import GeographicPosition, Waypoint
from mission.navigation import haversine_distance_m, spatial_distance_3d_m
from mission.trajectory_optimizer import CandidateRoute, PropulsionStrategy
from simulation.battery import BatteryModel, BatteryConfig
from simulation.hybrid_powertrain import (
    HybridPowertrain,
    HybridPowertrainConfig,
    HybridOperatingMode,
    ElectricMachineModel,
    PowerManagementController,
)
from simulation.flight_profile import FlightState
from simulation.scenario_generator import FastForwardConfig, ScenarioParameters, ScenarioType


def calculate_bearing_deg(p1: GeographicPosition, p2: GeographicPosition) -> float:
    """Calculates forward azimuth bearing in degrees [0, 360) from p1 to p2."""
    lat1 = math.radians(p1.latitude_deg)
    lat2 = math.radians(p2.latitude_deg)
    dlon = math.radians(p2.longitude_deg - p1.longitude_deg)

    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    brg = math.degrees(math.atan2(y, x))
    return (brg + 360.0) % 360.0


@dataclass
class ScenarioSimulationRecord:
    """
    Complete predictive record of a single scenario simulation run.
    Distinguishes unavailable values (None) from zero (0.0).
    """
    scenario_id: str
    scenario_type: str
    route_id: str
    destination_name: str

    # Completion & time
    is_completed: bool
    completion_status: str              # COMPLETED, DEPLETED_BATTERY, DEPLETED_FUEL, POWER_STARVATION, TIMEOUT
    simulated_elapsed_time_s: float
    estimated_arrival_time_s: Optional[float]
    total_distance_m: float
    completed_distance_m: float
    route_progress_pct: float

    # Battery predictions
    final_battery_soc_pct: float
    min_battery_soc_pct: float
    final_usable_battery_energy_wh: float
    final_battery_temp_c: float
    max_battery_temp_c: float
    final_battery_voltage_v: float
    min_battery_voltage_v: float

    # Fuel predictions
    final_fuel_remaining_liters: float
    total_fuel_consumed_liters: float

    # Propulsion & thermal
    mean_propulsion_power_req_w: float
    max_propulsion_power_req_w: float
    mean_propulsion_power_actual_w: float
    max_power_deficit_w: float
    cumulative_energy_deficit_wh: float
    max_engine_cht_c: float
    final_engine_oil_temp_c: float
    max_motor_temp_c: float

    # Arrival reserve margins
    arrival_fuel_reserve_liters: Optional[float]
    arrival_battery_soc_pct: Optional[float]
    arrival_reserve_margin_pct: Optional[float]

    # Threshold crossings
    has_fuel_reserve_breach: bool
    first_fuel_reserve_breach_s: Optional[float]
    has_battery_reserve_breach: bool
    first_battery_reserve_breach_s: Optional[float]
    has_battery_thermal_breach: bool
    first_battery_thermal_breach_s: Optional[float]
    has_engine_thermal_breach: bool
    first_engine_thermal_breach_s: Optional[float]
    has_power_deficit_breach: bool
    first_power_deficit_breach_s: Optional[float]
    has_thermal_violation: bool
    first_thermal_breach_s: Optional[float]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class FastForwardSimulationEngine:
    """
    Faster-Than-Real-Time UAV Multi-Scenario Predictive Simulation Engine.
    Executes isolated forward projections of candidate flight routes under uncertainty.
    """

    def __init__(self, config: Optional[Union[FastForwardConfig, Path, str, Dict[str, Any]]] = None):
        if config is None:
            default_path = Path(__file__).resolve().parent.parent / "configs" / "fast_forward_config.json"
            if default_path.exists():
                self.config = FastForwardConfig.from_json(default_path)
            else:
                self.config = FastForwardConfig()
                self.config.validate()
        elif isinstance(config, (str, Path)):
            self.config = FastForwardConfig.from_json(config)
        elif isinstance(config, dict):
            self.config = FastForwardConfig(**config)
            self.config.validate()
        elif isinstance(config, FastForwardConfig):
            config.validate()
            self.config = config
        else:
            raise TypeError(f"Invalid config type for FastForwardSimulationEngine: {type(config)}")

        # Shared reference configs
        self.battery_config = BatteryConfig()
        self.hybrid_config = HybridPowertrainConfig()

    def simulate_scenario(
        self,
        candidate_route: CandidateRoute,
        initial_fuel_liters: float,
        initial_soc_pct: float,
        initial_soh_pct: float = 100.0,
        initial_battery_temp_c: float = 25.0,
        initial_engine_cht_c: float = 175.0,
        initial_engine_oil_t_c: float = 88.0,
        initial_motor_temp_c: float = 25.0,
        scenario: Optional[ScenarioParameters] = None,
        dt_s: Optional[float] = None
    ) -> ScenarioSimulationRecord:
        """
        Executes a faster-than-real-time forward simulation of a candidate trajectory under scenario perturbations.
        Maintains complete state isolation from live digital twin models.
        """
        step_dt = float(dt_s if dt_s is not None else self.config.simulation_timestep_s)
        max_duration = float(self.config.max_simulation_duration_s)

        if scenario is None:
            scenario = ScenarioParameters(
                scenario_id="DEFAULT_NOMINAL",
                scenario_type=ScenarioType.NOMINAL,
                wind_speed_mps=0.0,
                wind_direction_deg=0.0,
                ambient_temp_delta_c=0.0,
                engine_power_multiplier=1.0,
                engine_fuel_flow_multiplier=1.0,
                battery_resistance_multiplier=1.0,
                battery_initial_soc_offset_pct=0.0,
                propulsion_power_multiplier=1.0,
                payload_mass_delta_kg=0.0
            )

        # 1. Instantiate isolated physical models
        # Create fresh battery model with scenario resistance multiplier
        b_cfg = BatteryConfig(
            nominal_internal_resistance_ohm=self.battery_config.nominal_internal_resistance_ohm * scenario.battery_resistance_multiplier,
            initial_soc_pct=max(1.0, min(100.0, initial_soc_pct + scenario.battery_initial_soc_offset_pct)),
            initial_soh_pct=initial_soh_pct,
            initial_temp_c=initial_battery_temp_c
        )
        battery = BatteryModel(config=b_cfg)
        em_model = ElectricMachineModel(config=self.hybrid_config)
        em_model.temperature_c = initial_motor_temp_c
        controller = PowerManagementController(config=self.hybrid_config)

        # 2. Initialize flight state & tracking variables
        waypoints = candidate_route.waypoints
        if len(waypoints) < 2:
            raise ValueError(f"Candidate route '{candidate_route.route_id}' has fewer than 2 waypoints.")

        current_fuel = float(initial_fuel_liters)
        total_fuel_consumed = 0.0

        engine_cht = float(initial_engine_cht_c)
        engine_oil_t = float(initial_engine_oil_t_c)
        max_cht = engine_cht
        max_battery_t = battery.temperature_c
        max_motor_t = em_model.temperature_c

        min_soc = battery.soc_pct
        min_voltage = 400.0

        p_req_samples: List[float] = []
        p_act_samples: List[float] = []
        max_deficit_w = 0.0
        cum_deficit_wh = 0.0

        # Safety threshold breach timestamps
        t_breach_fuel: Optional[float] = None
        t_breach_soc: Optional[float] = None
        t_breach_batt_t: Optional[float] = None
        t_breach_eng_t: Optional[float] = None
        t_breach_deficit: Optional[float] = None

        # Check initial threshold states
        if current_fuel < self.config.min_fuel_reserve_liters:
            t_breach_fuel = 0.0
        if battery.soc_pct < self.config.min_battery_reserve_pct:
            t_breach_soc = 0.0
        if battery.temperature_c >= self.config.critical_battery_temp_c:
            t_breach_batt_t = 0.0
        if engine_cht >= self.config.critical_engine_cht_c:
            t_breach_eng_t = 0.0

        # Precompute segment geometry
        segment_data = []
        total_route_distance_m = 0.0
        for idx in range(len(waypoints) - 1):
            wp_start = waypoints[idx]
            wp_end = waypoints[idx + 1]
            seg_dist = haversine_distance_m(wp_start, wp_end)
            seg_bearing = calculate_bearing_deg(wp_start, wp_end)
            alt_start = wp_start.altitude_m
            alt_end = wp_end.altitude_m
            total_route_distance_m += seg_dist
            segment_data.append({
                "start": wp_start,
                "end": wp_end,
                "distance_m": seg_dist,
                "bearing_deg": seg_bearing,
                "alt_start_m": alt_start,
                "alt_end_m": alt_end
            })

        elapsed_time_s = 0.0
        completed_dist_m = 0.0
        current_segment_idx = 0
        seg_progress_m = 0.0

        completion_status = "IN_PROGRESS"
        is_completed = False

        # Aircraft physical parameters
        base_mass_kg = self.config.base_uav_mass_kg + scenario.payload_mass_delta_kg
        g = 9.80665
        ld_ratio = 15.0
        eta_prop = 0.82
        nominal_tas_mps = 55.0

        last_batt_constrained = (battery.soc_pct <= 0.5 or battery.soc_pct >= 99.8)

        # 3. Main Fast-Forward Integration Loop
        while elapsed_time_s < max_duration and current_segment_idx < len(segment_data):
            seg = segment_data[current_segment_idx]
            seg_len = seg["distance_m"]

            # Interpolate current position along segment
            seg_frac = min(1.0, seg_progress_m / max(0.1, seg_len))
            alt_m = seg["alt_start_m"] + seg_frac * (seg["alt_end_m"] - seg["alt_start_m"])
            alt_ft = alt_m * 3.28084

            # ISA Atmosphere with Scenario Temperature Delta
            # T_ISA = 15.0 - 0.0065 * alt_m
            t_amb = (15.0 - 0.0065 * alt_m) + scenario.ambient_temp_delta_c
            t_amb_k = t_amb + 273.15
            p_amb_bar = (1.0 - 2.25577e-5 * alt_m) ** 5.25588
            rho_air = max(0.2, (p_amb_bar * 1e5) / (287.058 * t_amb_k))

            # Wind Vector Triangle along segment bearing
            bearing = seg["bearing_deg"]
            wind_angle_rad = math.radians(bearing - scenario.wind_direction_deg)
            v_headwind = scenario.wind_speed_mps * math.cos(wind_angle_rad)
            v_crosswind = scenario.wind_speed_mps * math.sin(wind_angle_rad)

            # Ground speed resolution
            v_tas = nominal_tas_mps
            v_along = math.sqrt(max(100.0, (v_tas ** 2) - (v_crosswind ** 2)))
            v_ground = max(10.0, v_along - v_headwind)

            # Segment climb rate
            dh_ds = (seg["alt_end_m"] - seg["alt_start_m"]) / max(1.0, seg_len)
            climb_rate_mps = dh_ds * v_ground

            # Aircraft Total Mass = dry + payload + remaining fuel (diesel ~0.835 kg/L)
            total_mass_kg = base_mass_kg + (current_fuel * 0.835)

            # Aerodynamic Propulsion Power Demand
            p_level = (total_mass_kg * g * v_tas) / (ld_ratio * eta_prop)
            p_climb = (total_mass_kg * g * max(0.0, climb_rate_mps)) / eta_prop
            p_req = (p_level + p_climb) * scenario.propulsion_power_multiplier

            # Engine Available Power at altitude with scenario degradation
            # Derating curve: 200 HP up to 10k ft, derates to 150 HP at 20k ft, 110 HP at 30k ft
            if alt_ft <= 10000.0:
                eng_rated_hp = 200.0
            elif alt_ft <= 20000.0:
                eng_rated_hp = 200.0 - 50.0 * ((alt_ft - 10000.0) / 10000.0)
            else:
                eng_rated_hp = max(70.0, 150.0 - 40.0 * ((alt_ft - 20000.0) / 10000.0))
            
            eng_max_avail_w = (eng_rated_hp * 745.7) * scenario.engine_power_multiplier

            # Powertrain Mode Selection
            mode = controller.select_operating_mode(
                propulsion_power_req_w=p_req,
                battery_soc_pct=battery.soc_pct,
                engine_max_power_w=eng_max_avail_w,
                battery_is_constrained=last_batt_constrained
            )

            # Split Power Demand
            if mode == HybridOperatingMode.ELECTRIC_ONLY:
                p_motor_mech = min(40000.0, p_req)
                p_eng_prop = 0.0
                fuel_lph = 1.8 * scenario.engine_fuel_flow_multiplier
                p_batt_req = p_motor_mech / (0.94 * 0.97)
                p_actual = p_motor_mech
                p_def = max(0.0, p_req - p_actual)
            elif mode == HybridOperatingMode.BATTERY_CHARGING:
                p_eng_prop = min(eng_max_avail_w, p_req)
                p_charge_shaft = min(20000.0, max(0.0, eng_max_avail_w - p_eng_prop))
                p_eng_tot = p_eng_prop + p_charge_shaft
                fuel_lph = min(40.0, 10.0 + (p_eng_tot / 75000.0) * 18.0) * scenario.engine_fuel_flow_multiplier
                p_batt_req = -p_charge_shaft * (0.92 * 0.97) # negative = charging
                p_actual = p_eng_prop
                p_def = max(0.0, p_req - p_actual)
            elif mode == HybridOperatingMode.MOTOR_ASSIST:
                p_eng_prop = eng_max_avail_w
                p_motor_req = max(0.0, p_req - p_eng_prop)
                p_motor_mech = min(40000.0, p_motor_req)
                fuel_lph = 36.5 * scenario.engine_fuel_flow_multiplier
                p_batt_req = p_motor_mech / (0.94 * 0.97)
                p_actual = p_eng_prop + p_motor_mech
                p_def = max(0.0, p_req - p_actual)
            else: # ENGINE_ONLY
                p_eng_prop = min(eng_max_avail_w, p_req)
                fuel_lph = min(38.0, 6.0 + (p_eng_prop / 50000.0) * 14.5) * scenario.engine_fuel_flow_multiplier
                p_batt_req = 0.0
                p_actual = p_eng_prop
                p_def = max(0.0, p_req - p_actual)

            # Step Battery Model
            batt_telem = battery.step(
                power_request_w=p_batt_req,
                ambient_temp_c=t_amb,
                dt_s=step_dt
            )
            last_batt_constrained = batt_telem.is_power_constrained

            # Step Electric Machine Model
            p_em_req = p_motor_mech if mode in (HybridOperatingMode.ELECTRIC_ONLY, HybridOperatingMode.MOTOR_ASSIST) else (-p_charge_shaft if mode == HybridOperatingMode.BATTERY_CHARGING else 0.0)
            em_state = em_model.step(
                mech_power_request_w=p_em_req,
                shaft_rpm=2800.0,
                ambient_temp_c=t_amb,
                dt_s=step_dt
            )

            # Step Engine Lumped Thermals & Fuel Burn
            # Reuses lumped thermal balance from MVEM (simulation/mvem.py)
            p_engine_mech = (p_eng_tot if mode == HybridOperatingMode.BATTERY_CHARGING else p_eng_prop)
            engine_hp = p_engine_mech / 745.7
            fuel_ratio = scenario.engine_fuel_flow_multiplier
            target_cht = (
                110.0 + 85.0 * (engine_hp / 200.0) * fuel_ratio +
                (t_amb - 15.0) * 0.25 -
                (v_tas - 30.0) * 0.35
            )
            alpha_cht = min(1.0, step_dt / 15.0)
            engine_cht += (target_cht - engine_cht) * alpha_cht

            # Oil thermal balance
            target_oil = 75.0 + 30.0 * (engine_hp / 200.0) + (t_amb - 15.0) * 0.20
            alpha_oil = min(1.0, step_dt / 30.0)
            engine_oil_t += (target_oil - engine_oil_t) * alpha_oil

            # Decrement Fuel
            step_fuel_l = (fuel_lph / 3600.0) * step_dt
            current_fuel = max(0.0, current_fuel - step_fuel_l)
            total_fuel_consumed += step_fuel_l

            # Track Extremes & Deficits
            min_soc = min(min_soc, battery.soc_pct)
            min_voltage = min(min_voltage, batt_telem.terminal_voltage_v)
            max_battery_t = max(max_battery_t, battery.temperature_c)
            max_cht = max(max_cht, engine_cht)
            max_motor_t = max(max_motor_t, em_model.temperature_c)

            p_req_samples.append(p_req)
            p_act_samples.append(p_actual)
            max_deficit_w = max(max_deficit_w, p_def)
            if p_def > 0.0:
                cum_deficit_wh += (p_def * (step_dt / 3600.0))

            # Monitor First Breach Timestamps
            if t_breach_fuel is None and current_fuel < self.config.min_fuel_reserve_liters:
                t_breach_fuel = round(elapsed_time_s, 1)
            if t_breach_soc is None and battery.soc_pct < self.config.min_battery_reserve_pct:
                t_breach_soc = round(elapsed_time_s, 1)
            if t_breach_batt_t is None and battery.temperature_c >= self.config.critical_battery_temp_c:
                t_breach_batt_t = round(elapsed_time_s, 1)
            if t_breach_eng_t is None and engine_cht >= self.config.critical_engine_cht_c:
                t_breach_eng_t = round(elapsed_time_s, 1)
            if t_breach_deficit is None and p_def > self.config.max_permitted_power_deficit_w:
                t_breach_deficit = round(elapsed_time_s, 1)

            # Advance Distance along segment
            step_dist_m = v_ground * step_dt
            seg_progress_m += step_dist_m
            completed_dist_m += step_dist_m
            elapsed_time_s += step_dt

            # Check segment completion
            if seg_progress_m >= seg_len:
                current_segment_idx += 1
                seg_progress_m = 0.0

            # Termination Checks
            if current_fuel <= 0.01:
                # Fuel starvation
                if battery.soc_pct <= 5.0:
                    completion_status = "DEPLETED_FUEL"
                    break
            if battery.soc_pct <= 0.5 and p_def > 15000.0:
                completion_status = "DEPLETED_BATTERY"
                break
            if p_def > 25000.0 and elapsed_time_s > 60.0:
                completion_status = "POWER_STARVATION"
                break

        # Check if full trajectory reached
        if current_segment_idx >= len(segment_data):
            is_completed = True
            completion_status = "COMPLETED"
            arrival_fuel = current_fuel
            arrival_soc = battery.soc_pct
            # Calculate arrival reserve margin
            usable_f = max(0.0, arrival_fuel - self.config.min_fuel_reserve_liters)
            usable_b = max(0.0, (arrival_soc - self.config.min_battery_reserve_pct) / 100.0 * 18000.0)
            arrival_margin = ((usable_f * 2700.0 + usable_b) / 50000.0) * 100.0
            eta_arrival_s = elapsed_time_s
        else:
            arrival_fuel = None
            arrival_soc = None
            arrival_margin = None
            eta_arrival_s = None
            if completion_status == "IN_PROGRESS":
                completion_status = "TIMEOUT"

        # Determine consolidated thermal breach
        first_therm: Optional[float] = None
        if t_breach_batt_t is not None and t_breach_eng_t is not None:
            first_therm = min(t_breach_batt_t, t_breach_eng_t)
        elif t_breach_batt_t is not None:
            first_therm = t_breach_batt_t
        elif t_breach_eng_t is not None:
            first_therm = t_breach_eng_t

        mean_p_req = (sum(p_req_samples) / len(p_req_samples)) if p_req_samples else 0.0
        mean_p_act = (sum(p_act_samples) / len(p_act_samples)) if p_act_samples else 0.0
        max_p_req = max(p_req_samples) if p_req_samples else 0.0
        prog_pct = min(100.0, (completed_dist_m / max(1.0, total_route_distance_m)) * 100.0)

        return ScenarioSimulationRecord(
            scenario_id=scenario.scenario_id,
            scenario_type=scenario.scenario_type.value,
            route_id=candidate_route.route_id,
            destination_name=candidate_route.destination_name,
            is_completed=is_completed,
            completion_status=completion_status,
            simulated_elapsed_time_s=round(elapsed_time_s, 1),
            estimated_arrival_time_s=round(eta_arrival_s, 1) if eta_arrival_s is not None else None,
            total_distance_m=round(total_route_distance_m, 1),
            completed_distance_m=round(completed_dist_m, 1),
            route_progress_pct=round(prog_pct, 1),
            final_battery_soc_pct=round(battery.soc_pct, 2),
            min_battery_soc_pct=round(min_soc, 2),
            final_usable_battery_energy_wh=round((battery.soc_pct / 100.0) * battery.config.nominal_capacity_ah * (battery.soh_pct / 100.0) * battery.config.nominal_voltage_v, 1),
            final_battery_temp_c=round(battery.temperature_c, 1),
            max_battery_temp_c=round(max_battery_t, 1),
            final_battery_voltage_v=round(battery.get_open_circuit_voltage(battery.soc_pct), 1),
            min_battery_voltage_v=round(min_voltage, 1),
            final_fuel_remaining_liters=round(current_fuel, 2),
            total_fuel_consumed_liters=round(total_fuel_consumed, 2),
            mean_propulsion_power_req_w=round(mean_p_req, 1),
            max_propulsion_power_req_w=round(max_p_req, 1),
            mean_propulsion_power_actual_w=round(mean_p_act, 1),
            max_power_deficit_w=round(max_deficit_w, 1),
            cumulative_energy_deficit_wh=round(cum_deficit_wh, 2),
            max_engine_cht_c=round(max_cht, 1),
            final_engine_oil_temp_c=round(engine_oil_t, 1),
            max_motor_temp_c=round(max_motor_t, 1),
            arrival_fuel_reserve_liters=round(arrival_fuel, 2) if arrival_fuel is not None else None,
            arrival_battery_soc_pct=round(arrival_soc, 2) if arrival_soc is not None else None,
            arrival_reserve_margin_pct=round(arrival_margin, 1) if arrival_margin is not None else None,
            has_fuel_reserve_breach=(t_breach_fuel is not None),
            first_fuel_reserve_breach_s=t_breach_fuel,
            has_battery_reserve_breach=(t_breach_soc is not None),
            first_battery_reserve_breach_s=t_breach_soc,
            has_battery_thermal_breach=(t_breach_batt_t is not None),
            first_battery_thermal_breach_s=t_breach_batt_t,
            has_engine_thermal_breach=(t_breach_eng_t is not None),
            first_engine_thermal_breach_s=t_breach_eng_t,
            has_power_deficit_breach=(t_breach_deficit is not None),
            first_power_deficit_breach_s=t_breach_deficit,
            has_thermal_violation=(first_therm is not None),
            first_thermal_breach_s=first_therm
        )
