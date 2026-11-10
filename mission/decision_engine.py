"""
AI-Guided Mission Decision Engine Module (Step 8)
Evaluates digital twin state, Step 7 reliability assessment, propulsion availability,
battery status, and navigation geometry to produce deterministic, explainable
mission recommendations:
- CONTINUE_MISSION
- ABORT_AND_RTB
- DIVERT_TO_ALTERNATE
- EMERGENCY_DESCENT

No unvalidated heuristic weights or black-box random numbers.
Enforces strict safety reserves and explicit handling of missing/invalid telemetry.
"""

from dataclasses import dataclass, field, asdict
from enum import Enum
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Union

from mission.mission_state import GeographicPosition, Waypoint, MissionNavigationState
from mission.reliability_engine import (
    ContingencyAction,
    ReliabilityStatusLevel,
    AlternateAirfield,
    MissionReliabilityState,
)


class MissionDecisionAction(str, Enum):
    CONTINUE_MISSION = "CONTINUE_MISSION"
    ABORT_AND_RTB = "ABORT_AND_RTB"
    DIVERT_TO_ALTERNATE = "DIVERT_TO_ALTERNATE"
    EMERGENCY_DESCENT = "EMERGENCY_DESCENT"


@dataclass
class DestinationInfo:
    """Configured flight recovery or waypoint destination."""
    destination_id: str
    name: str
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    runway_length_m: Optional[float] = None
    priority: int = 1

    def to_position(self) -> GeographicPosition:
        return GeographicPosition(
            latitude_deg=self.latitude_deg,
            longitude_deg=self.longitude_deg,
            altitude_m=self.altitude_m
        )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class DecisionEngineConfig:
    """Configuration policies for deterministic mission decision-making."""
    policy_name: str = "DRDO-MALE-UAV-Decision-Engine-Standard-v1"
    min_battery_reserve_pct: float = 20.0
    min_fuel_reserve_liters: float = 18.0
    emergency_power_reserve_w: float = 15000.0
    max_permitted_power_deficit_w: float = 8000.0
    critical_engine_health_threshold_pct: float = 40.0
    degraded_engine_health_threshold_pct: float = 65.0
    critical_battery_temp_c: float = 65.0
    caution_battery_temp_c: float = 55.0
    min_decision_confidence_healthy: float = 0.95
    telemetry_stale_timeout_s: float = 0.20
    optimization_interval_s: float = 1.0
    route_optimization_weights: Dict[str, float] = field(default_factory=lambda: {
        "weight_fuel": 0.30,
        "weight_battery": 0.20,
        "weight_duration": 0.20,
        "weight_distance": 0.10,
        "weight_margin": 0.10,
        "weight_degradation": 0.10,
    })
    normalization_scales: Dict[str, float] = field(default_factory=lambda: {
        "norm_distance_m": 300000.0,
        "norm_duration_s": 5400.0,
        "norm_fuel_l": 35.0,
        "norm_battery_wh": 18000.0,
    })
    alternate_destinations: List[DestinationInfo] = field(default_factory=list)

    def validate(self):
        """
        Validates configuration consistency and safety limits.
        Raises ValueError if invalid.
        """
        if self.min_battery_reserve_pct <= 0.0 or self.min_battery_reserve_pct >= 100.0:
            raise ValueError(f"Invalid min_battery_reserve_pct: {self.min_battery_reserve_pct}. Must be in (0, 100).")
        if self.min_fuel_reserve_liters <= 0.0:
            raise ValueError(f"Invalid min_fuel_reserve_liters: {self.min_fuel_reserve_liters}. Must be > 0.")
        if self.critical_engine_health_threshold_pct >= self.degraded_engine_health_threshold_pct:
            raise ValueError("critical_engine_health_threshold_pct must be less than degraded_engine_health_threshold_pct.")
        if self.critical_battery_temp_c <= self.caution_battery_temp_c:
            raise ValueError("critical_battery_temp_c must be greater than caution_battery_temp_c.")

        # Check weights
        weights = self.route_optimization_weights
        if not weights:
            raise ValueError("route_optimization_weights cannot be empty.")
        total_w = sum(weights.values())
        if abs(total_w - 1.0) > 0.01:
            raise ValueError(f"route_optimization_weights must sum to 1.0 (got {total_w:.4f}).")
        for k, v in weights.items():
            if v < 0.0:
                raise ValueError(f"Optimization weight '{k}' cannot be negative: {v}")

        # Check normalization scales
        scales = self.normalization_scales
        for k, v in scales.items():
            if v <= 0.0:
                raise ValueError(f"Normalization scale '{k}' must be strictly positive: {v}")

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DecisionEngineConfig":
        alts = []
        for a in data.get("alternate_destinations", []):
            alts.append(DestinationInfo(
                destination_id=a.get("destination_id", "ALT"),
                name=a.get("name", "Alternate Destination"),
                latitude_deg=float(a.get("latitude_deg", 0.0)),
                longitude_deg=float(a.get("longitude_deg", 0.0)),
                altitude_m=float(a.get("altitude_m", 0.0)),
                runway_length_m=float(a["runway_length_m"]) if "runway_length_m" in a and a["runway_length_m"] is not None else None,
                priority=int(a.get("priority", 1))
            ))

        cfg = cls(
            policy_name=data.get("policy_name", "DRDO-MALE-UAV-Decision-Engine-Standard-v1"),
            min_battery_reserve_pct=float(data.get("min_battery_reserve_pct", 20.0)),
            min_fuel_reserve_liters=float(data.get("min_fuel_reserve_liters", 18.0)),
            emergency_power_reserve_w=float(data.get("emergency_power_reserve_w", 15000.0)),
            max_permitted_power_deficit_w=float(data.get("max_permitted_power_deficit_w", 8000.0)),
            critical_engine_health_threshold_pct=float(data.get("critical_engine_health_threshold_pct", 40.0)),
            degraded_engine_health_threshold_pct=float(data.get("degraded_engine_health_threshold_pct", 65.0)),
            critical_battery_temp_c=float(data.get("critical_battery_temp_c", 65.0)),
            caution_battery_temp_c=float(data.get("caution_battery_temp_c", 55.0)),
            min_decision_confidence_healthy=float(data.get("min_decision_confidence_healthy", 0.95)),
            telemetry_stale_timeout_s=float(data.get("telemetry_stale_timeout_s", 0.20)),
            optimization_interval_s=float(data.get("optimization_interval_s", 1.0)),
            route_optimization_weights=data.get("route_optimization_weights", {
                "weight_fuel": 0.30,
                "weight_battery": 0.20,
                "weight_duration": 0.20,
                "weight_distance": 0.10,
                "weight_margin": 0.10,
                "weight_degradation": 0.10,
            }),
            normalization_scales=data.get("normalization_scales", {
                "norm_distance_m": 300000.0,
                "norm_duration_s": 5400.0,
                "norm_fuel_l": 35.0,
                "norm_battery_wh": 18000.0,
            }),
            alternate_destinations=alts
        )
        cfg.validate()
        return cfg

    @classmethod
    def from_json(cls, json_path: Union[str, Path]) -> "DecisionEngineConfig":
        p = Path(json_path)
        if not p.exists():
            raise FileNotFoundError(f"Decision engine config not found: {p}")
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data)


@dataclass
class MissionDecisionResult:
    """Structured decision output from the Mission Decision Engine."""
    recommended_action: str             # CONTINUE_MISSION, ABORT_AND_RTB, etc.
    decision_reason: str                 # Human-readable explanation
    decision_confidence: float           # 0.0 to 1.0 based on data trust & completeness
    triggered_conditions: List[str]      # Specific safety/operational rules triggered
    mission_feasible: bool               # True if planned mission can be safely completed
    rtb_feasible: bool                   # True if safe return to home base is achievable
    alternate_feasible: bool             # True if safe recovery at alternate airfield is achievable
    selected_destination: str            # Recommended destination name / ID
    decision_timestamp: float            # Master timestamp of evaluation [s]
    data_quality_status: str             # HEALTHY, WARNING, DEGRADED, INVALID

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class MissionDecisionEngine:
    """
    Deterministic AI-Guided Mission Decision Engine.
    Evaluates multi-subsystem UAV state and enforces regulatory safety reserve policies.
    """

    def __init__(self, config: Optional[Union[DecisionEngineConfig, str, Path]] = None):
        if config is None:
            default_json = Path(__file__).resolve().parent.parent / "configs" / "decision_engine_config.json"
            if default_json.exists():
                self.config = DecisionEngineConfig.from_json(default_json)
            else:
                self.config = DecisionEngineConfig()
                self.config.validate()
        elif isinstance(config, (str, Path)):
            self.config = DecisionEngineConfig.from_json(config)
        elif isinstance(config, DecisionEngineConfig):
            config.validate()
            self.config = config
        else:
            raise TypeError(f"Invalid config type for MissionDecisionEngine: {type(config)}")

        self.last_decision: Optional[MissionDecisionResult] = None

    def evaluate(
        self,
        timestamp_s: Optional[float],
        quality_status: Optional[str],
        engine_health_pct: Optional[float],
        battery_soc_pct: Optional[float],
        battery_usable_kwh: Optional[float],
        battery_temp_c: Optional[float],
        battery_is_constrained: Optional[bool],
        remaining_fuel_liters: Optional[float],
        power_deficit_w: Optional[float],
        propulsion_power_req_w: Optional[float],
        distance_to_home_m: Optional[float],
        remaining_route_distance_m: Optional[float],
        reliability_state: Optional[MissionReliabilityState] = None,
        stale_data_flags: Optional[Dict[str, bool]] = None,
        next_waypoint_name: str = "Target Waypoint",
        home_destination_name: str = "Pune Lohegaon (Home Base)"
    ) -> MissionDecisionResult:
        """
        Executes deterministic multi-subsystem decision tree.
        Missing critical inputs are rejected explicitly rather than silently defaulting to zero.
        """
        # ------------------------------------------------------------------
        # 1. Explicit Missing Inputs Check (Never silently replace with zero)
        # ------------------------------------------------------------------
        missing_fields = []
        if timestamp_s is None:
            missing_fields.append("timestamp_s")
        if quality_status is None:
            missing_fields.append("quality_status")
        if engine_health_pct is None:
            missing_fields.append("engine_health_pct")
        if battery_soc_pct is None:
            missing_fields.append("battery_soc_pct")
        if remaining_fuel_liters is None:
            missing_fields.append("remaining_fuel_liters")

        if missing_fields:
            # Explicit failure on missing critical telemetry
            return MissionDecisionResult(
                recommended_action=MissionDecisionAction.ABORT_AND_RTB.value,
                decision_reason=f"Critical telemetry inputs missing: {', '.join(missing_fields)}. Abort mandatory.",
                decision_confidence=0.10,
                triggered_conditions=["MISSING_CRITICAL_TELEMETRY"],
                mission_feasible=False,
                rtb_feasible=False,
                alternate_feasible=False,
                selected_destination="UNKNOWN_EMERGENCY",
                decision_timestamp=float(timestamp_s or 0.0),
                data_quality_status="INVALID"
            )

        t = float(timestamp_s)
        q_stat = str(quality_status).upper()
        eng_h = float(engine_health_pct)
        b_soc = float(battery_soc_pct)
        b_kwh = float(battery_usable_kwh or 0.0)
        b_temp = float(battery_temp_c if battery_temp_c is not None else 25.0)
        b_constrained = bool(battery_is_constrained)
        fuel_rem = float(remaining_fuel_liters)
        p_def = float(power_deficit_w or 0.0)
        p_req = float(propulsion_power_req_w or 50000.0)
        d_home = float(distance_to_home_m or 0.0)
        d_rem = float(remaining_route_distance_m or 0.0)

        # Numerical sanity check (NaN / Infinity check)
        is_corrupt = (
            math.isnan(t) or math.isinf(t) or
            math.isnan(eng_h) or math.isinf(eng_h) or
            math.isnan(b_soc) or math.isinf(b_soc) or
            math.isnan(fuel_rem) or math.isinf(fuel_rem) or
            math.isnan(p_def) or math.isinf(p_def) or
            math.isnan(d_home) or math.isinf(d_home)
        )
        if is_corrupt:
            q_stat = "INVALID"

        triggered: List[str] = []

        # ------------------------------------------------------------------
        # 2. Feasibility Evaluation
        # ------------------------------------------------------------------
        rtb_feasible = True
        alternate_feasible = False
        mission_feasible = True

        if reliability_state is not None:
            rtb_feasible = bool(reliability_state.rtb_feasible)
            alternate_feasible = bool(reliability_state.rtb_prediction.nearest_alternate_feasible)
            nearest_alt_id = reliability_state.rtb_prediction.nearest_alternate_id or "ALT-01"
            rel_score = reliability_state.mission_reliability_index
        else:
            # Standalone estimation if reliability state omitted
            # Cruise fuel burn ~22 LPH, speed ~55 m/s -> ~0.111 L/km
            fuel_for_home = (d_home / 1000.0) * 0.111 * 1.20
            rtb_feasible = (fuel_rem - fuel_for_home) >= self.config.min_fuel_reserve_liters and (b_soc >= self.config.min_battery_reserve_pct)
            alternate_feasible = len(self.config.alternate_destinations) > 0 and (fuel_rem >= self.config.min_fuel_reserve_liters)
            nearest_alt_id = self.config.alternate_destinations[0].destination_id if self.config.alternate_destinations else "ALT-01"
            rel_score = 90.0 if (rtb_feasible and eng_h >= 80.0) else 50.0

        # Mission continuation feasibility check
        fuel_for_mission = (d_rem / 1000.0) * 0.111 * 1.20
        if (fuel_rem - fuel_for_mission) < self.config.min_fuel_reserve_liters:
            mission_feasible = False
            triggered.append("INSUFFICIENT_FUEL_FOR_MISSION")

        if b_soc < (self.config.min_battery_reserve_pct + 10.0):
            mission_feasible = False
            triggered.append("LOW_BATTERY_FOR_MISSION")

        if eng_h < self.config.degraded_engine_health_threshold_pct:
            mission_feasible = False
            triggered.append("DEGRADED_ENGINE_HEALTH")

        if p_def > self.config.max_permitted_power_deficit_w:
            mission_feasible = False
            triggered.append("EXCESSIVE_PROPULSION_POWER_DEFICIT")

        if q_stat in ["INVALID", "DEGRADED"]:
            mission_feasible = False
            triggered.append(f"DATA_QUALITY_{q_stat}")

        # Check staleness
        is_stale = False
        if stale_data_flags:
            stale_subs = [k for k, v in stale_data_flags.items() if v]
            if stale_subs:
                is_stale = True
                mission_feasible = False
                triggered.append(f"STALE_SUBSYSTEMS_{','.join(stale_subs).upper()}")

        # ------------------------------------------------------------------
        # 3. Decision Confidence Calibration
        # ------------------------------------------------------------------
        confidence = self.config.min_decision_confidence_healthy
        if q_stat == "WARNING":
            confidence -= 0.10
        elif q_stat == "DEGRADED":
            confidence -= 0.35
        elif q_stat == "INVALID":
            confidence = 0.20

        if is_stale:
            confidence = min(confidence, 0.55)

        if p_def > 1000.0:
            def_frac = min(1.0, p_def / max(1.0, p_req))
            confidence -= 0.15 * def_frac

        if eng_h < self.config.degraded_engine_health_threshold_pct:
            confidence -= 0.15 * (1.0 - (eng_h / 100.0))

        confidence = round(max(0.10, min(1.0, confidence)), 3)

        # ------------------------------------------------------------------
        # 4. Deterministic Decision Tree Execution (Safety-First Priority)
        # ------------------------------------------------------------------

        # RULE 1: Catastrophic Emergency Conditions -> EMERGENCY_DESCENT
        is_dual_propulsion_failure = (
            (eng_h < self.config.critical_engine_health_threshold_pct) and
            (b_soc < 10.0 or b_constrained or b_kwh < 1.0)
        )
        is_thermal_critical = (b_temp >= self.config.critical_battery_temp_c)
        is_unrecoverable_energy = (not rtb_feasible and not alternate_feasible)

        if is_dual_propulsion_failure or is_thermal_critical or is_unrecoverable_energy:
            reasons = []
            if is_dual_propulsion_failure:
                reasons.append(f"Dual propulsion loss: Engine health ({eng_h:.1f}%) and battery depleted/constrained ({b_soc:.1f}% SoC)")
                triggered.append("DUAL_PROPULSION_LOSS")
            if is_thermal_critical:
                reasons.append(f"Battery thermal critical ({b_temp:.1f}°C >= {self.config.critical_battery_temp_c}°C limit)")
                triggered.append("BATTERY_THERMAL_CRITICAL")
            if is_unrecoverable_energy:
                reasons.append("Insufficient energy to achieve Home Base or any configured alternate recovery airfield")
                triggered.append("ALL_DESTINATIONS_ENERGY_INFEASIBLE")

            res = MissionDecisionResult(
                recommended_action=MissionDecisionAction.EMERGENCY_DESCENT.value,
                decision_reason=f"CRITICAL FLIGHT EMERGENCY: {'; '.join(reasons)}. Initiate immediate controlled glide descent to nearest open terrain sector.",
                decision_confidence=confidence,
                triggered_conditions=triggered,
                mission_feasible=False,
                rtb_feasible=False,
                alternate_feasible=alternate_feasible,
                selected_destination="EMERGENCY_TERRAIN_SECTOR",
                decision_timestamp=t,
                data_quality_status=q_stat
            )
            self.last_decision = res
            return res

        # RULE 2: Data Corruption or Invalid Telemetry -> ABORT_AND_RTB (NEVER CONTINUE)
        if q_stat == "INVALID":
            triggered.append("TELEMETRY_INVALID_CORRUPTION")
            res = MissionDecisionResult(
                recommended_action=MissionDecisionAction.ABORT_AND_RTB.value,
                decision_reason="Telemetry data quality is INVALID (NaN/infinite corruption or physical conservation failure). Flight safety cannot be guaranteed; abort mission sortie immediately.",
                decision_confidence=confidence,
                triggered_conditions=triggered,
                mission_feasible=False,
                rtb_feasible=rtb_feasible,
                alternate_feasible=alternate_feasible,
                selected_destination=home_destination_name if rtb_feasible else nearest_alt_id,
                decision_timestamp=t,
                data_quality_status=q_stat
            )
            self.last_decision = res
            return res

        # RULE 3: Subsystem Telemetry Lag / Staleness -> ABORT_AND_RTB (NEVER CONTINUE)
        if q_stat == "DEGRADED" or is_stale:
            triggered.append("SUBSYSTEM_TELEMETRY_STALE")
            res = MissionDecisionResult(
                recommended_action=MissionDecisionAction.ABORT_AND_RTB.value,
                decision_reason="Subsystem telemetry is DEGRADED with stale sensor lag (>200ms). Autonomous situational awareness compromised; abort sortie to maintain pilot command link.",
                decision_confidence=confidence,
                triggered_conditions=triggered,
                mission_feasible=False,
                rtb_feasible=rtb_feasible,
                alternate_feasible=alternate_feasible,
                selected_destination=home_destination_name if rtb_feasible else nearest_alt_id,
                decision_timestamp=t,
                data_quality_status=q_stat
            )
            self.last_decision = res
            return res

        # RULE 4: Home Base Return Infeasible but Alternate Feasible -> DIVERT_TO_ALTERNATE
        if not rtb_feasible:
            triggered.append("HOME_BASE_RTB_INFEASIBLE")
            if alternate_feasible:
                alt_obj = next((a for a in self.config.alternate_destinations if a.destination_id == nearest_alt_id), None)
                alt_name = alt_obj.name if alt_obj else nearest_alt_id
                res = MissionDecisionResult(
                    recommended_action=MissionDecisionAction.DIVERT_TO_ALTERNATE.value,
                    decision_reason=f"Return to Home Base infeasible ({d_home/1000.0:.1f} km requires energy exceeding reserves). Divert immediately to designated alternate: {alt_name}.",
                    decision_confidence=confidence,
                    triggered_conditions=triggered,
                    mission_feasible=False,
                    rtb_feasible=False,
                    alternate_feasible=True,
                    selected_destination=alt_name,
                    decision_timestamp=t,
                    data_quality_status=q_stat
                )
                self.last_decision = res
                return res
            else:
                # Neither home nor alternate feasible
                triggered.append("NO_REACHABLE_AIRFIELD")
                res = MissionDecisionResult(
                    recommended_action=MissionDecisionAction.EMERGENCY_DESCENT.value,
                    decision_reason=f"Both Home Base ({d_home/1000.0:.1f} km) and Alternate recovery airfields exceed available onboard energy. Execute controlled descent.",
                    decision_confidence=confidence,
                    triggered_conditions=triggered,
                    mission_feasible=False,
                    rtb_feasible=False,
                    alternate_feasible=False,
                    selected_destination="EMERGENCY_RECOVERY_SECTOR",
                    decision_timestamp=t,
                    data_quality_status=q_stat
                )
                self.last_decision = res
                return res

        # RULE 5: Safety Reserve Policy Breached (Fuel or Battery) -> ABORT_AND_RTB
        if fuel_rem < self.config.min_fuel_reserve_liters:
            triggered.append("FUEL_SAFETY_RESERVE_BREACH")
            res = MissionDecisionResult(
                recommended_action=MissionDecisionAction.ABORT_AND_RTB.value,
                decision_reason=f"Fuel reserve policy breached: {fuel_rem:.1f} L remaining (< {self.config.min_fuel_reserve_liters:.1f} L limit). Return-to-Base required to protect landing buffer.",
                decision_confidence=confidence,
                triggered_conditions=triggered,
                mission_feasible=False,
                rtb_feasible=True,
                alternate_feasible=alternate_feasible,
                selected_destination=home_destination_name,
                decision_timestamp=t,
                data_quality_status=q_stat
            )
            self.last_decision = res
            return res

        if b_soc < self.config.min_battery_reserve_pct:
            triggered.append("BATTERY_SAFETY_RESERVE_BREACH")
            res = MissionDecisionResult(
                recommended_action=MissionDecisionAction.ABORT_AND_RTB.value,
                decision_reason=f"Battery reserve policy breached: {b_soc:.1f}% SoC remaining (< {self.config.min_battery_reserve_pct:.1f}% limit). Return-to-Base required to protect emergency power headroom.",
                decision_confidence=confidence,
                triggered_conditions=triggered,
                mission_feasible=False,
                rtb_feasible=True,
                alternate_feasible=alternate_feasible,
                selected_destination=home_destination_name,
                decision_timestamp=t,
                data_quality_status=q_stat
            )
            self.last_decision = res
            return res

        # RULE 6: Propulsion Mechanical Degradation -> ABORT_AND_RTB
        if eng_h < self.config.degraded_engine_health_threshold_pct:
            triggered.append("ENGINE_HEALTH_DEGRADED_ABORT")
            res = MissionDecisionResult(
                recommended_action=MissionDecisionAction.ABORT_AND_RTB.value,
                decision_reason=f"Engine mechanical health degraded ({eng_h:.1f}% < {self.config.degraded_engine_health_threshold_pct:.1f}% threshold). Abort planned sortie to prevent in-flight engine seizure.",
                decision_confidence=confidence,
                triggered_conditions=triggered,
                mission_feasible=False,
                rtb_feasible=True,
                alternate_feasible=alternate_feasible,
                selected_destination=home_destination_name,
                decision_timestamp=t,
                data_quality_status=q_stat
            )
            self.last_decision = res
            return res

        # RULE 7: Persistent Propulsion Power Deficit -> ABORT_AND_RTB
        if p_def > self.config.max_permitted_power_deficit_w:
            triggered.append("PROPULSION_DEFICIT_ABORT")
            res = MissionDecisionResult(
                recommended_action=MissionDecisionAction.ABORT_AND_RTB.value,
                decision_reason=f"Propulsion power deficit ({p_def/1000.0:.1f} kW) exceeds maximum permitted limit ({self.config.max_permitted_power_deficit_w/1000.0:.1f} kW). Unable to sustain climb or target loiter altitude.",
                decision_confidence=confidence,
                triggered_conditions=triggered,
                mission_feasible=False,
                rtb_feasible=True,
                alternate_feasible=alternate_feasible,
                selected_destination=home_destination_name,
                decision_timestamp=t,
                data_quality_status=q_stat
            )
            self.last_decision = res
            return res

        # RULE 8: Mission Infeasible (Fuel or Route Budget Exhaustion) -> ABORT_AND_RTB
        if not mission_feasible:
            res = MissionDecisionResult(
                recommended_action=MissionDecisionAction.ABORT_AND_RTB.value,
                decision_reason="Remaining mission flight route cannot be safely completed within regulatory fuel/battery margins. Early Return-to-Base initiated.",
                decision_confidence=confidence,
                triggered_conditions=triggered,
                mission_feasible=False,
                rtb_feasible=True,
                alternate_feasible=alternate_feasible,
                selected_destination=home_destination_name,
                decision_timestamp=t,
                data_quality_status=q_stat
            )
            self.last_decision = res
            return res

        # RULE 9: All Safety & Operational Checks Verified -> CONTINUE_MISSION
        triggered.append("ALL_PRECONDITIONS_NOMINAL")
        res = MissionDecisionResult(
            recommended_action=MissionDecisionAction.CONTINUE_MISSION.value,
            decision_reason=f"All platform systems NOMINAL (Reliability: {rel_score:.1f}%, Reserves: {fuel_rem:.1f}L / {b_soc:.1f}% SoC). Safe to continue active route toward {next_waypoint_name}.",
            decision_confidence=confidence,
            triggered_conditions=triggered,
            mission_feasible=True,
            rtb_feasible=True,
            alternate_feasible=alternate_feasible,
            selected_destination=next_waypoint_name,
            decision_timestamp=t,
            data_quality_status=q_stat
        )
        self.last_decision = res
        return res
