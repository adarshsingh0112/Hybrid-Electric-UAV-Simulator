"""
Mission Reliability Scoring & Dynamic Contingency Analysis Module (Step 7)
Provides:
1. Dynamic Return-to-Base (RTB) energy and fuel requirement prediction.
2. Physics-anchored, explainable Mission Reliability Index (0–100%).
3. Deterministic dynamic contingency recommendations:
   - CONTINUE_MISSION
   - ABORT_AND_RTB
   - DIVERT_TO_ALTERNATE
   - EMERGENCY_DESCENT
4. Configurable safety reserve management.
"""

from dataclasses import dataclass, field, asdict
from enum import Enum
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Union

from mission.mission_state import GeographicPosition
from mission.navigation import haversine_distance_m


class ContingencyAction(str, Enum):
    CONTINUE_MISSION = "CONTINUE_MISSION"
    ABORT_AND_RTB = "ABORT_AND_RTB"
    DIVERT_TO_ALTERNATE = "DIVERT_TO_ALTERNATE"
    EMERGENCY_DESCENT = "EMERGENCY_DESCENT"


class ReliabilityStatusLevel(str, Enum):
    NOMINAL = "NOMINAL"       # Score >= 80% (Safe to continue normal mission)
    ADVISORY = "ADVISORY"     # Score 65–79% (Subsystem caution, monitor trends)
    CAUTION = "CAUTION"       # Score 50–64% (Reserve or health degraded, prepare RTB)
    CRITICAL = "CRITICAL"     # Score < 50% (Immediate abort, divert, or emergency action)


@dataclass
class AlternateAirfield:
    """Designated emergency / contingency recovery base."""
    base_id: str
    name: str
    latitude_deg: float
    longitude_deg: float
    altitude_m: float

    def to_position(self) -> GeographicPosition:
        return GeographicPosition(
            latitude_deg=self.latitude_deg,
            longitude_deg=self.longitude_deg,
            altitude_m=self.altitude_m
        )


@dataclass
class ReliabilityConfig:
    """Configurable reserve policies, aircraft parameters, and alternate airfields."""
    policy_name: str = "DRDO-MALE-UAV-Safety-Reserve-Policy-Standard"
    min_battery_reserve_pct: float = 20.0        # Minimum battery SoC % upon touchdown
    min_fuel_reserve_liters: float = 18.0        # Minimum fuel reserve (~45 min loiter)
    emergency_power_reserve_w: float = 15000.0   # Emergency reserve power [W]
    rtb_energy_margin_factor: float = 1.20       # 20% contingency buffer on RTB energy
    nominal_fuel_capacity_liters: float = 150.0  # Total fuel tank volume [liters]
    uav_mass_kg: float = 1100.0                  # Normal take-off mass [kg]
    nominal_cruise_speed_mps: float = 55.0       # True airspeed [m/s]
    cruise_lift_to_drag_ratio: float = 15.0      # Aerodynamic efficiency L/D
    propeller_efficiency: float = 0.82           # Propeller propulsion efficiency
    nominal_cruise_fuel_burn_lph: float = 22.0   # Cruise fuel burn [liters/hour]
    max_fuel_burn_lph: float = 38.0              # Max continuous fuel burn [liters/hour]
    alternate_recovery_bases: List[AlternateAirfield] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ReliabilityConfig":
        alts = []
        for b in data.get("alternate_recovery_bases", []):
            alts.append(AlternateAirfield(
                base_id=b.get("base_id", "ALT"),
                name=b.get("name", "Alternate Base"),
                latitude_deg=float(b.get("latitude_deg", 0.0)),
                longitude_deg=float(b.get("longitude_deg", 0.0)),
                altitude_m=float(b.get("altitude_m", 0.0))
            ))
        return cls(
            policy_name=data.get("policy_name", "DRDO-MALE-UAV-Safety-Reserve-Policy-Standard"),
            min_battery_reserve_pct=float(data.get("min_battery_reserve_pct", 20.0)),
            min_fuel_reserve_liters=float(data.get("min_fuel_reserve_liters", 18.0)),
            emergency_power_reserve_w=float(data.get("emergency_power_reserve_w", 15000.0)),
            rtb_energy_margin_factor=float(data.get("rtb_energy_margin_factor", 1.20)),
            nominal_fuel_capacity_liters=float(data.get("nominal_fuel_capacity_liters", 150.0)),
            uav_mass_kg=float(data.get("uav_mass_kg", 1100.0)),
            nominal_cruise_speed_mps=float(data.get("nominal_cruise_speed_mps", 55.0)),
            cruise_lift_to_drag_ratio=float(data.get("cruise_lift_to_drag_ratio", 15.0)),
            propeller_efficiency=float(data.get("propeller_efficiency", 0.82)),
            nominal_cruise_fuel_burn_lph=float(data.get("nominal_cruise_fuel_burn_lph", 22.0)),
            max_fuel_burn_lph=float(data.get("max_fuel_burn_lph", 38.0)),
            alternate_recovery_bases=alts
        )

    @classmethod
    def from_json(cls, path: Union[str, Path]) -> "ReliabilityConfig":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))


@dataclass
class RTBPredictionResult:
    """Detailed Return-to-Base trajectory and energy consumption estimate."""
    distance_to_home_m: float
    estimated_duration_s: float
    estimated_power_w: float
    estimated_energy_wh: float
    estimated_fuel_liters: float
    available_battery_wh: float
    available_usable_battery_wh: float
    available_fuel_liters: float
    available_usable_fuel_liters: float
    available_total_energy_wh: float
    energy_margin_pct: float
    rtb_feasible: bool
    limiting_factor: str
    nearest_alternate_id: Optional[str] = None
    nearest_alternate_distance_m: Optional[float] = None
    nearest_alternate_feasible: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ReliabilityScoreBreakdown:
    """Individual explainable factor sub-indices (0–100)."""
    energy_reserve_factor: float
    propulsion_health_factor: float
    power_headroom_factor: float
    telemetry_quality_factor: float
    mission_progress_factor: float
    composite_score: float

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MissionReliabilityState:
    """Subsystem state model for mission reliability and dynamic contingency analysis."""
    timestamp_s: float
    mission_reliability_index: float           # 0.0 to 100.0
    reliability_status: str                     # NOMINAL, ADVISORY, CAUTION, CRITICAL
    rtb_feasible: bool                          # True if safe return transit is achievable
    estimated_rtb_time_s: float                 # Transit time back to home base [s]
    estimated_rtb_energy_wh: float              # Total shaft energy required [Wh]
    estimated_rtb_fuel_liters: float            # Fuel required for return transit [L]
    available_fuel_liters: float                # Remaining fuel onboard [L]
    available_battery_wh: float                 # Remaining usable battery energy [Wh]
    contingency_recommendation: str             # CONTINUE_MISSION, ABORT_AND_RTB, etc.
    contingency_reason: str                     # Plain-language explanation for pilots
    available_energy_margin_pct: float          # Percentage buffer above safety reserve
    score_breakdown: ReliabilityScoreBreakdown
    rtb_prediction: RTBPredictionResult
    provenance: str = "SIMULATED"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class MissionReliabilityEngine:
    """
    Continuous Mission Reliability Scoring & Dynamic Contingency Decision Engine.
    Executes physically grounded evaluation of return-to-base feasibility and platform safety.
    """

    def __init__(self, config: Optional[Union[ReliabilityConfig, str, Path]] = None):
        if config is None:
            default_json = Path(__file__).resolve().parent.parent / "configs" / "reliability_config.json"
            if default_json.exists():
                self.config = ReliabilityConfig.from_json(default_json)
            else:
                self.config = ReliabilityConfig()
        elif isinstance(config, (str, Path)):
            self.config = ReliabilityConfig.from_json(config)
        elif isinstance(config, ReliabilityConfig):
            self.config = config
        else:
            self.config = ReliabilityConfig()

        # Dynamic state tracking
        self.remaining_fuel_liters: float = self.config.nominal_fuel_capacity_liters
        self.last_timestamp_s: Optional[float] = None
        self.last_decision: ContingencyAction = ContingencyAction.CONTINUE_MISSION
        self.last_decision_reason: str = "Nominal mission flight profile."

    def reset(self, initial_fuel_liters: Optional[float] = None):
        """Resets engine state and replenishes fuel reserves."""
        self.remaining_fuel_liters = (
            initial_fuel_liters if initial_fuel_liters is not None
            else self.config.nominal_fuel_capacity_liters
        )
        self.last_timestamp_s = None
        self.last_decision = ContingencyAction.CONTINUE_MISSION
        self.last_decision_reason = "Nominal mission flight profile."

    def predict_rtb_energy(
        self,
        current_pos: GeographicPosition,
        home_pos: GeographicPosition,
        current_altitude_m: float,
        ground_speed_mps: float,
        battery_usable_wh: float,
        battery_soc_pct: float,
        remaining_fuel_liters: float,
        dt_s: float = 0.05
    ) -> RTBPredictionResult:
        """
        Calculates dynamic Return-to-Base (RTB) transit time, aerodynamic power, and energy requirement.
        """
        # 1. Geographic Distance to Home Base
        dist_home_m = haversine_distance_m(current_pos, home_pos)

        # 2. Cruise Speed Safe Handling (Guards against zero/negative speeds)
        speed = ground_speed_mps if (ground_speed_mps is not None and ground_speed_mps > 5.0) else self.config.nominal_cruise_speed_mps
        est_duration_s = dist_home_m / max(1.0, speed)

        # 3. Aerodynamic Power Requirement
        # Steady-level cruise power: P_shaft = (m * g / (L/D)) * V / eta_prop
        m = self.config.uav_mass_kg
        g = 9.80665
        l_over_d = max(5.0, self.config.cruise_lift_to_drag_ratio)
        eta_prop = max(0.5, self.config.propeller_efficiency)

        drag_n = (m * g) / l_over_d
        p_thrust_w = drag_n * speed
        p_shaft_cruise_w = p_thrust_w / eta_prop

        # Elevation compensation: descending from high altitude assists descent
        alt_diff_m = home_pos.altitude_m - current_altitude_m
        if est_duration_s > 1.0 and abs(alt_diff_m) > 10.0:
            p_potential_w = (m * g * alt_diff_m) / est_duration_s
            p_req_w = max(20000.0, p_shaft_cruise_w + p_potential_w)
        else:
            p_req_w = p_shaft_cruise_w

        # Total Shaft Energy (Wh) required with policy margin factor
        raw_energy_wh = (p_req_w * (est_duration_s / 3600.0))
        est_energy_wh = raw_energy_wh * self.config.rtb_energy_margin_factor

        # 4. Fuel Required for RTB (Liters)
        # Scaled by power demand relative to nominal cruise power
        burn_rate_lph = self.config.nominal_cruise_fuel_burn_lph * (p_req_w / max(1.0, p_shaft_cruise_w))
        burn_rate_lph = max(10.0, min(self.config.max_fuel_burn_lph, burn_rate_lph))
        est_fuel_liters = (burn_rate_lph * (est_duration_s / 3600.0)) * self.config.rtb_energy_margin_factor

        # 5. Usable Reserves Available
        # Fuel above minimum reserve
        usable_fuel_liters = max(0.0, remaining_fuel_liters - self.config.min_fuel_reserve_liters)

        # Battery above minimum reserve (20% SoC buffer)
        pack_total_wh = 18000.0 # Standard 18 kWh pack
        min_batt_wh = pack_total_wh * (self.config.min_battery_reserve_pct / 100.0)
        usable_batt_wh = max(0.0, battery_usable_wh - min_batt_wh)

        # Combined equivalent usable energy (Diesel energy content ~ 9,800 Wh/L * 0.38 thermal eff = 3724 Wh/L)
        diesel_eff_wh_per_l = 3724.0
        total_avail_wh = (usable_fuel_liters * diesel_eff_wh_per_l) + usable_batt_wh

        # Energy Margin % above required RTB energy
        if est_energy_wh > 0.1:
            margin_pct = ((total_avail_wh - est_energy_wh) / est_energy_wh) * 100.0
        else:
            margin_pct = 100.0

        # 6. RTB Feasibility Evaluation
        # Feasible if remaining fuel covers transit with reserve, or hybrid battery covers deficit
        rtb_feasible = True
        limiting_factor = "NONE_FEASIBLE"

        if usable_fuel_liters < est_fuel_liters:
            # Check if battery can buffer the fuel deficit
            fuel_deficit_l = est_fuel_liters - usable_fuel_liters
            energy_deficit_wh = fuel_deficit_l * diesel_eff_wh_per_l
            if usable_batt_wh < energy_deficit_wh:
                rtb_feasible = False
                limiting_factor = "INSUFFICIENT_FUEL_AND_BATTERY"
            else:
                limiting_factor = "FUEL_DEFICIT_COVERED_BY_BATTERY"
        elif battery_soc_pct < self.config.min_battery_reserve_pct:
            limiting_factor = "BATTERY_RESERVE_DEPLETED"

        # 7. Check Nearest Alternate Airfield
        nearest_alt_id = None
        nearest_alt_dist_m = None
        nearest_alt_feasible = False

        if self.config.alternate_recovery_bases:
            min_d = float("inf")
            best_alt = None
            for alt in self.config.alternate_recovery_bases:
                d = haversine_distance_m(current_pos, alt.to_position())
                if d < min_d:
                    min_d = d
                    best_alt = alt

            if best_alt:
                nearest_alt_id = best_alt.base_id
                nearest_alt_dist_m = min_d
                # Feasible if alternate distance is substantially closer than home
                alt_energy_req_wh = (p_req_w * ((min_d / max(1.0, speed)) / 3600.0)) * self.config.rtb_energy_margin_factor
                nearest_alt_feasible = (total_avail_wh >= alt_energy_req_wh)

        return RTBPredictionResult(
            distance_to_home_m=round(dist_home_m, 1),
            estimated_duration_s=round(est_duration_s, 1),
            estimated_power_w=round(p_req_w, 1),
            estimated_energy_wh=round(est_energy_wh, 1),
            estimated_fuel_liters=round(est_fuel_liters, 2),
            available_battery_wh=round(battery_usable_wh, 1),
            available_usable_battery_wh=round(usable_batt_wh, 1),
            available_fuel_liters=round(remaining_fuel_liters, 2),
            available_usable_fuel_liters=round(usable_fuel_liters, 2),
            available_total_energy_wh=round(total_avail_wh, 1),
            energy_margin_pct=round(margin_pct, 1),
            rtb_feasible=rtb_feasible,
            limiting_factor=limiting_factor,
            nearest_alternate_id=nearest_alt_id,
            nearest_alternate_distance_m=round(nearest_alt_dist_m, 1) if nearest_alt_dist_m else None,
            nearest_alternate_feasible=nearest_alt_feasible
        )

    def evaluate(
        self,
        timestamp_s: float,
        current_pos: GeographicPosition,
        home_pos: GeographicPosition,
        altitude_m: float,
        ground_speed_mps: float,
        engine_health_pct: float,
        engine_power_avail_w: float,
        fuel_flow_lph: float,
        battery_energy_wh: float,
        battery_soc_pct: float,
        battery_soh_pct: float,
        battery_temp_c: float,
        battery_is_constrained: bool,
        electric_motor_power_avail_w: float,
        current_power_demand_w: float,
        power_deficit_w: float,
        mission_progress_pct: float,
        quality_status: str,
        stale_data_flags: Optional[Dict[str, bool]] = None,
        dt_s: float = 0.05
    ) -> MissionReliabilityState:
        """
        Executes continuous Mission Reliability Scoring and Dynamic Contingency Analysis.
        """
        # Numerical Sanity & NaN/Inf Protection
        def sanitize(val: Optional[float], default: float, min_val: float, max_val: float) -> float:
            if val is None or math.isnan(val) or math.isinf(val):
                return default
            return max(min_val, min(max_val, float(val)))

        t = sanitize(timestamp_s, 0.0, 0.0, 1e7)
        alt = sanitize(altitude_m, 1000.0, -100.0, 25000.0)
        speed = sanitize(ground_speed_mps, 55.0, 0.0, 200.0)
        eng_h = sanitize(engine_health_pct, 100.0, 0.0, 100.0)
        eng_p_avail = sanitize(engine_power_avail_w, 149000.0, 0.0, 200000.0)
        ff_lph = sanitize(fuel_flow_lph, 22.0, 0.0, 100.0)
        b_wh = sanitize(battery_energy_wh, 18000.0, 0.0, 25000.0)
        b_soc = sanitize(battery_soc_pct, 100.0, 0.0, 100.0)
        b_soh = sanitize(battery_soh_pct, 100.0, 0.0, 100.0)
        b_temp = sanitize(battery_temp_c, 25.0, -40.0, 120.0)
        em_p_avail = sanitize(electric_motor_power_avail_w, 40000.0, 0.0, 50000.0)
        p_demand = sanitize(current_power_demand_w, 60000.0, 0.0, 250000.0)
        p_deficit = sanitize(power_deficit_w, 0.0, 0.0, 200000.0)
        prog_pct = sanitize(mission_progress_pct, 0.0, 0.0, 100.0)
        q_stat = quality_status if quality_status in ["HEALTHY", "WARNING", "DEGRADED", "INVALID"] else "INVALID"

        # Update Fuel Consumption Integration
        if self.last_timestamp_s is not None and t > self.last_timestamp_s:
            delta_t = t - self.last_timestamp_s
            fuel_burned = (ff_lph / 3600.0) * delta_t
            self.remaining_fuel_liters = max(0.0, self.remaining_fuel_liters - fuel_burned)
        self.last_timestamp_s = t

        # 1. Predict Return-to-Base (RTB) Trajectory & Consumption
        rtb = self.predict_rtb_energy(
            current_pos=current_pos,
            home_pos=home_pos,
            current_altitude_m=alt,
            ground_speed_mps=speed,
            battery_usable_wh=b_wh,
            battery_soc_pct=b_soc,
            remaining_fuel_liters=self.remaining_fuel_liters,
            dt_s=dt_s
        )

        # 2. Factor 1: Energy Reserve Factor (0–100)
        # Evaluates available fuel and battery buffers against required RTB budget with policy margin
        margin = rtb.energy_margin_pct
        if margin >= 50.0:
            f_energy = 100.0
        elif margin >= 0.0:
            f_energy = 60.0 + 40.0 * (margin / 50.0)
        else:
            # Negative margin (insufficient reserve)
            f_energy = max(0.0, 60.0 + margin * 0.6)

        # Penalty if fuel below minimum safety reserve policy
        if self.remaining_fuel_liters < self.config.min_fuel_reserve_liters:
            deficit_frac = (self.config.min_fuel_reserve_liters - self.remaining_fuel_liters) / self.config.min_fuel_reserve_liters
            f_energy = min(f_energy, max(10.0, 40.0 * (1.0 - deficit_frac)))

        # Penalty if battery SoC below minimum policy
        if b_soc < self.config.min_battery_reserve_pct:
            soc_frac = b_soc / self.config.min_battery_reserve_pct
            f_energy = min(f_energy, max(10.0, 50.0 * soc_frac))

        # 3. Factor 2: Propulsion Subsystem Health Factor (0–100)
        # Weighted composite of Engine Mechanical Health (70%) and Battery SoH (30%)
        # Non-linear penalty if engine health degrades below caution threshold (70%)
        if eng_h >= 85.0:
            eng_score = eng_h
        elif eng_h >= 65.0:
            eng_score = 50.0 + (eng_h - 65.0) * 1.75
        else:
            eng_score = max(0.0, eng_h * 0.70)

        f_health = 0.70 * eng_score + 0.30 * b_soh

        # Battery thermal penalty if approaching safety limits
        if b_temp >= 55.0:
            f_health = min(f_health, max(15.0, 100.0 - (b_temp - 55.0) * 8.0))

        # 4. Factor 3: Power Headroom Factor (0–100)
        # Ratio of available combined propulsion capacity to requested demand
        total_power_avail = eng_p_avail + em_p_avail
        if p_demand > 1.0:
            headroom_ratio = total_power_avail / p_demand
        else:
            headroom_ratio = 2.0

        if headroom_ratio >= 1.30 and p_deficit < 100.0:
            f_power = 100.0
        elif headroom_ratio >= 1.0 and p_deficit < 1000.0:
            f_power = 70.0 + 30.0 * ((headroom_ratio - 1.0) / 0.30)
        else:
            # Power deficit active!
            deficit_ratio = p_deficit / max(1.0, p_demand)
            f_power = max(10.0, 60.0 * (1.0 - deficit_ratio))

        if p_deficit > 8000.0:
            deficit_sev = min(1.0, (p_deficit - 8000.0) / 40000.0)
            f_power = min(f_power, max(10.0, 38.0 - 20.0 * deficit_sev))

        if battery_is_constrained:
            f_power = min(f_power, 55.0)

        # 5. Factor 4: Telemetry Data Trust Factor (0–100)
        if q_stat == "HEALTHY":
            f_telemetry = 100.0
        elif q_stat == "WARNING":
            f_telemetry = 85.0
        elif q_stat == "DEGRADED":
            f_telemetry = 50.0
        else: # INVALID
            f_telemetry = 0.0

        if stale_data_flags and any(stale_data_flags.values()):
            stale_count = sum(1 for v in stale_data_flags.values() if v)
            f_telemetry = max(10.0, f_telemetry - stale_count * 15.0)

        # 6. Factor 5: Mission Progress & Distance Factor (0–100)
        # Proximity to recovery advantage
        dist_factor = math.exp(-rtb.distance_to_home_m / 300000.0) # 300 km characteristic scale
        f_progress = round(min(100.0, 40.0 + 60.0 * dist_factor), 1)

        # 7. Weighted Composite Score
        # Weights: Energy 35%, Health 30%, Power 15%, Telemetry 15%, Progress 5%
        raw_composite = (
            0.35 * f_energy +
            0.30 * f_health +
            0.15 * f_power +
            0.15 * f_telemetry +
            0.05 * f_progress
        )

        # 8. Strict Safety Hard-Clamps
        composite = raw_composite
        if q_stat == "INVALID":
            # Corrupted data can NEVER produce a trusted reliability score
            composite = min(10.0, composite)
        elif not rtb.rtb_feasible:
            # Cannot safely return to home base
            composite = min(40.0, composite)
        elif eng_h < 40.0 or b_temp >= 60.0:
            # Severe subsystem failure
            composite = min(35.0, composite)
        elif self.remaining_fuel_liters < self.config.min_fuel_reserve_liters or b_soc < self.config.min_battery_reserve_pct:
            # Safety reserve violated
            composite = min(50.0, composite)
        elif eng_h < 65.0:
            # Degraded propulsion health
            composite = min(60.0, composite)
        elif p_deficit > 8000.0:
            # Persistent propulsion power deficit
            composite = min(55.0, composite)
        elif q_stat == "DEGRADED":
            # Subsystem telemetry lag / stale data
            composite = min(60.0, composite)

        composite_score = round(max(0.0, min(100.0, composite)), 1)

        # Status level classification
        if composite_score >= 80.0:
            rel_status = ReliabilityStatusLevel.NOMINAL.value
        elif composite_score >= 65.0:
            rel_status = ReliabilityStatusLevel.ADVISORY.value
        elif composite_score >= 50.0:
            rel_status = ReliabilityStatusLevel.CAUTION.value
        else:
            rel_status = ReliabilityStatusLevel.CRITICAL.value

        score_breakdown = ReliabilityScoreBreakdown(
            energy_reserve_factor=round(f_energy, 1),
            propulsion_health_factor=round(f_health, 1),
            power_headroom_factor=round(f_power, 1),
            telemetry_quality_factor=round(f_telemetry, 1),
            mission_progress_factor=round(f_progress, 1),
            composite_score=composite_score
        )

        # 9. Dynamic Contingency Decision Analysis (Explainable Rules)
        recommendation, reason = self._determine_contingency(
            q_stat=q_stat,
            eng_h=eng_h,
            b_soc=b_soc,
            b_wh=b_wh,
            b_temp=b_temp,
            b_constrained=battery_is_constrained,
            p_deficit=p_deficit,
            remaining_fuel=self.remaining_fuel_liters,
            rtb=rtb,
            score=composite_score
        )

        self.last_decision = recommendation
        self.last_decision_reason = reason

        return MissionReliabilityState(
            timestamp_s=round(t, 2),
            mission_reliability_index=composite_score,
            reliability_status=rel_status,
            rtb_feasible=rtb.rtb_feasible,
            estimated_rtb_time_s=rtb.estimated_duration_s,
            estimated_rtb_energy_wh=rtb.estimated_energy_wh,
            estimated_rtb_fuel_liters=rtb.estimated_fuel_liters,
            available_fuel_liters=round(self.remaining_fuel_liters, 2),
            available_battery_wh=round(b_wh, 1),
            contingency_recommendation=recommendation.value,
            contingency_reason=reason,
            available_energy_margin_pct=rtb.energy_margin_pct,
            score_breakdown=score_breakdown,
            rtb_prediction=rtb,
            provenance="SIMULATED"
        )

    def _determine_contingency(
        self,
        q_stat: str,
        eng_h: float,
        b_soc: float,
        b_wh: float,
        b_temp: float,
        b_constrained: bool,
        p_deficit: float,
        remaining_fuel: float,
        rtb: RTBPredictionResult,
        score: float
    ) -> Tuple[ContingencyAction, str]:
        """
        Deterministic, safety-first contingency logic tree with plain-language explanations.
        """
        # Rule 1: Catastrophic Emergency Conditions -> EMERGENCY_DESCENT
        if (eng_h < 20.0 and (b_soc < 10.0 or b_constrained)) or b_temp >= 65.0:
            return (
                ContingencyAction.EMERGENCY_DESCENT,
                f"Dual propulsion loss or battery thermal critical ({b_temp:.1f}°C). Execute emergency descent to nearest clear terrain."
            )

        # Rule 2: Corrupted or Missing Safety Telemetry -> ABORT_AND_RTB
        if q_stat == "INVALID":
            return (
                ContingencyAction.ABORT_AND_RTB,
                "Telemetry quality is INVALID (data corruption or power conservation violation). Flight safety cannot be verified; abort mission immediately."
            )

        # Rule 3: Insufficient Energy to Home Base -> DIVERT_TO_ALTERNATE or EMERGENCY_DESCENT
        if not rtb.rtb_feasible:
            if rtb.nearest_alternate_feasible and rtb.nearest_alternate_id:
                dist_km = (rtb.nearest_alternate_distance_m or 0.0) / 1000.0
                return (
                    ContingencyAction.DIVERT_TO_ALTERNATE,
                    f"Return to Home Base infeasible ({rtb.distance_to_home_m/1000.0:.1f} km). Divert to {rtb.nearest_alternate_id} ({dist_km:.1f} km) within energy reserves."
                )
            else:
                return (
                    ContingencyAction.EMERGENCY_DESCENT,
                    f"Insufficient energy for Home Base or Alternate recovery. Execute controlled glide to emergency recovery sector."
                )

        # Rule 4: Critical Fuel or Battery Reserve Policy Violation -> ABORT_AND_RTB
        if remaining_fuel < self.config.min_fuel_reserve_liters:
            return (
                ContingencyAction.ABORT_AND_RTB,
                f"Fuel reserve violated: {remaining_fuel:.1f} L remaining (< {self.config.min_fuel_reserve_liters:.1f} L policy limit). RTB while energy reserves remain."
            )

        if b_soc < self.config.min_battery_reserve_pct:
            return (
                ContingencyAction.ABORT_AND_RTB,
                f"Battery reserve violated: {b_soc:.1f}% SoC (< {self.config.min_battery_reserve_pct:.1f}% policy limit). RTB to protect emergency buffer."
            )

        # Rule 5: Degraded Propulsion Health or Severe Power Deficit -> ABORT_AND_RTB
        if eng_h < 65.0:
            return (
                ContingencyAction.ABORT_AND_RTB,
                f"Engine mechanical health degraded ({eng_h:.1f}%). Abort mission sortie to prevent uncontained engine failure."
            )

        if p_deficit > 8000.0:
            return (
                ContingencyAction.ABORT_AND_RTB,
                f"Persistent propulsion power deficit ({p_deficit/1000.0:.1f} kW). Unable to sustain climb or target loiter altitude."
            )

        # Rule 6: Telemetry Degraded / High Lag -> ABORT_AND_RTB
        if q_stat == "DEGRADED":
            return (
                ContingencyAction.ABORT_AND_RTB,
                "Subsystem telemetry is DEGRADED (stale data lag > 200 ms). Operator advisory: abort mission to restore GCS link."
            )

        # Rule 7: Low Energy Buffer (< 15% margin) -> ABORT_AND_RTB
        if rtb.energy_margin_pct < 15.0:
            return (
                ContingencyAction.ABORT_AND_RTB,
                f"Energy margin low ({rtb.energy_margin_pct:.1f}% above reserve). Return transit recommended before reserve erosion."
            )

        # Rule 8: All Safety Preconditions Verified -> CONTINUE_MISSION
        return (
            ContingencyAction.CONTINUE_MISSION,
            f"All flight systems NOMINAL (Reliability: {score:.1f}%, RTB Margin: +{rtb.energy_margin_pct:.1f}%). Safe to continue mission route."
        )
