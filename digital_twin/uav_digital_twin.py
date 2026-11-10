"""
Unified Real-Time UAV Digital Twin State & Orchestration Layer
Maintains an authoritative, synchronized, validated virtual representation of the UAV platform.

Architecture:
1. Subsystem State Models: Engine, Battery, Electric Machine, Hybrid Powertrain, Environment, Mission Navigation.
2. Unified UAV Digital Twin State (UAVDigitalTwinState) & Telemetry Snapshot (UAVDigitalTwinTelemetry).
3. Digital Twin Quality Status (HEALTHY, WARNING, DEGRADED, INVALID) — evaluates data trust & synchronization.
4. Validation & Cross-System Consistency Layer (UAVDigitalTwinValidator).
5. Bounded Chronological History Buffer (UAVDigitalTwinHistoryBuffer).
"""

from collections import deque
from dataclasses import dataclass, field, asdict
from enum import Enum
import json
import math
from typing import Dict, List, Optional, Tuple, Any, Union

from simulation.flight_profile import FlightState
from simulation.mvem import EngineState
from simulation.battery import BatteryTelemetry, BatteryChargeState
from simulation.hybrid_powertrain import (
    HybridPowertrainTelemetry,
    HybridOperatingMode,
    HybridControlMode,
    ElectricMachineState
)
from mission.mission_state import (
    GeographicPosition,
    Waypoint,
    MissionNavigationState,
    MissionNavigationSubsystemState
)
from mission.navigation import SpatialNavigationEngine, haversine_distance_m
from mission.reliability_engine import (
    MissionReliabilityEngine,
    MissionReliabilityState,
)
from mission.decision_engine import (
    MissionDecisionEngine,
    MissionDecisionResult,
    MissionDecisionAction,
    DecisionEngineConfig,
)
from mission.trajectory_optimizer import (
    TrajectoryOptimizer,
    TrajectoryOptimizationTelemetry,
    CandidateRoute,
)
from mission.predictive_risk_analyzer import (
    PredictiveRiskAnalyzer,
    PredictiveRiskReport,
    PredictiveSimulationTelemetry,
)
from mission.mission_rerouting_manager import (
    MissionReroutingManager,
    MissionReroutingTelemetry,
    ProposalStatus,
    RouteProposal,
)



class DigitalTwinQualityStatus(str, Enum):
    HEALTHY = "HEALTHY"    # All subsystems synchronized, verified cross-system consistency, 0 errors.
    WARNING = "WARNING"    # Non-critical numerical mismatch or minor operational warning.
    DEGRADED = "DEGRADED"  # Stale telemetry detected from one or more subsystems.
    INVALID = "INVALID"    # Severe data corruption (NaN, Infinity, impossible bounds, power balance violation).


@dataclass
class EngineSubsystemState:
    timestamp_s: float
    rpm: float
    power_hp: float
    power_w: float
    throttle_pct: float
    manifold_pressure_bar: float
    oil_pressure_bar: float
    oil_temp_c: float
    coolant_temp_c: float
    fuel_flow_lph: float
    egt_c: List[float]
    cht_c: List[float]
    vibration_rms_g: float


@dataclass
class BatterySubsystemState:
    timestamp_s: float
    soc_pct: float
    soh_pct: float
    terminal_voltage_v: float
    open_circuit_voltage_v: float
    internal_resistance_ohm: float
    current_a: float
    power_demand_w: float
    power_actual_w: float
    power_loss_w: float
    usable_energy_wh: float
    usable_energy_kwh: float
    temperature_c: float
    charge_state: str
    is_power_constrained: bool


@dataclass
class ElectricMachineSubsystemState:
    timestamp_s: float
    operating_mode: str        # MOTOR, GENERATOR, IDLE
    mechanical_power_w: float
    electrical_power_w: float
    torque_nm: float
    rpm: float
    efficiency: float
    power_loss_w: float
    temperature_c: float
    is_thermal_warning: bool


@dataclass
class HybridPowertrainSubsystemState:
    timestamp_s: float
    operating_mode: str        # ENGINE_ONLY, ELECTRIC_ONLY, MOTOR_ASSIST, BATTERY_CHARGING
    control_mode: str          # AUTO or MANUAL
    propulsion_power_req_w: float
    propulsion_power_actual_w: float
    propulsion_power_deficit_w: float
    engine_power_contribution_w: float
    electric_power_contribution_w: float
    energy_flow_direction: str
    is_system_power_constrained: bool


@dataclass
class EnvironmentSubsystemState:
    timestamp_s: float
    altitude_ft: float
    altitude_m: float
    ambient_temp_c: float
    ambient_pressure_bar: float
    air_density_kgpm3: float
    airspeed_mps: float
    flight_phase: str


@dataclass
class UAVDigitalTwinState:
    """
    Authoritative, synchronized State Model representing the complete UAV virtual entity.
    """
    timestamp_s: float
    environment: EnvironmentSubsystemState
    engine: EngineSubsystemState
    battery: BatterySubsystemState
    electric_machine: ElectricMachineSubsystemState
    powertrain: HybridPowertrainSubsystemState
    mission: MissionNavigationSubsystemState
    reliability: Optional[MissionReliabilityState] = None
    decision: Optional[MissionDecisionResult] = None
    trajectory: Optional[TrajectoryOptimizationTelemetry] = None
    prediction: Optional[PredictiveSimulationTelemetry] = None
    rerouting: Optional[MissionReroutingTelemetry] = None
    quality_status: DigitalTwinQualityStatus = DigitalTwinQualityStatus.HEALTHY
    validation_warnings: List[str] = field(default_factory=list)
    validation_errors: List[str] = field(default_factory=list)
    stale_data_flags: Dict[str, bool] = field(default_factory=lambda: {
        "engine": False,
        "battery": False,
        "electric_machine": False,
        "powertrain": False,
        "environment": False,
        "mission": False,
        "reliability": False,
        "decision": False,
        "trajectory": False,
        "prediction": False,
        "rerouting": False,
    })
    provenance: str = "SIMULATED"

    def to_dict(self) -> Dict[str, Any]:
        """Serializes complete state into nested dictionary."""
        d = asdict(self)
        d["quality_status"] = self.quality_status.value
        return d


@dataclass
class UAVDigitalTwinTelemetry:
    """
    Immutable/Safely exportable Telemetry Snapshot created from UAVDigitalTwinState.
    """
    timestamp_s: float
    quality_status: str
    environment: EnvironmentSubsystemState
    engine: EngineSubsystemState
    battery: BatterySubsystemState
    electric_machine: ElectricMachineSubsystemState
    powertrain: HybridPowertrainSubsystemState
    mission: MissionNavigationSubsystemState
    validation_warnings: List[str]
    validation_errors: List[str]
    stale_data_flags: Dict[str, bool]
    provenance: str = "SIMULATED"
    reliability: Optional[MissionReliabilityState] = None
    decision: Optional[MissionDecisionResult] = None
    trajectory: Optional[TrajectoryOptimizationTelemetry] = None
    prediction: Optional[PredictiveSimulationTelemetry] = None
    prediction_status: Optional[str] = None
    prediction_timestamp: Optional[float] = None
    prediction_scenario_count: Optional[int] = None
    predicted_mission_completion_rate: Optional[float] = None
    predicted_rtb_feasibility_rate: Optional[float] = None
    predicted_battery_depletion_rate: Optional[float] = None
    predicted_engine_thermal_risk: Optional[float] = None
    predicted_battery_thermal_risk: Optional[float] = None
    predicted_fuel_reserve_violation_rate: Optional[float] = None
    predicted_energy_reserve_p05: Optional[float] = None
    predicted_energy_reserve_p95: Optional[float] = None
    recommended_predictive_route: Optional[str] = None
    prediction_computation_time_ms: Optional[float] = None
    rerouting: Optional[MissionReroutingTelemetry] = None
    reroute_status: Optional[str] = None
    reroute_pending_count: Optional[int] = None
    reroute_active_proposal_id: Optional[str] = None
    reroute_proposal_status: Optional[str] = None
    reroute_active_route_id: Optional[str] = None
    reroute_proposed_route_id: Optional[str] = None
    reroute_proposed_destination: Optional[str] = None
    reroute_contingency_detected: Optional[bool] = None
    reroute_attention_required: Optional[bool] = None
    reroute_last_operator_action: Optional[str] = None


    def to_dict(self) -> Dict[str, Any]:
        """Serializes telemetry snapshot to dictionary."""
        return asdict(self)

    def to_json(self) -> str:
        """Serializes telemetry snapshot to JSON string."""
        return json.dumps(self.to_dict())


class UAVDigitalTwinValidator:
    """
    Validation and Cross-System Consistency Layer.
    Verifies numerical sanity, timestamp synchronization, physical conservation laws,
    and geographic navigation constraints across all subsystems.
    """

    @staticmethod
    def validate(state: UAVDigitalTwinState, stale_timeout_s: float = 0.20) -> DigitalTwinQualityStatus:
        """
        Executes strict verification rules against synchronized UAVDigitalTwinState.
        """
        warnings = []
        errors = []
        quality = DigitalTwinQualityStatus.HEALTHY

        master_t = state.timestamp_s

        # -------------------------------------------------------------
        # 1. Numerical Sanity & NaN / Infinity Check
        # -------------------------------------------------------------
        def check_num(val: float, name: str):
            if val is None or math.isnan(val) or math.isinf(val):
                errors.append(f"Invalid numerical value in {name}: {val}")

        check_num(master_t, "master timestamp")
        check_num(state.battery.soc_pct, "battery SoC")
        check_num(state.battery.soh_pct, "battery SoH")
        check_num(state.battery.usable_energy_wh, "battery energy")
        check_num(state.engine.power_w, "engine power")
        check_num(state.electric_machine.mechanical_power_w, "motor power")
        check_num(state.powertrain.propulsion_power_actual_w, "propulsion actual power")
        check_num(state.powertrain.propulsion_power_req_w, "propulsion requested power")

        # Check navigation numeric fields
        nav = state.mission
        check_num(nav.current_position.latitude_deg, "navigation latitude")
        check_num(nav.current_position.longitude_deg, "navigation longitude")
        check_num(nav.current_position.altitude_m, "navigation altitude")
        check_num(nav.distance_to_next_waypoint_m, "distance to next waypoint")
        check_num(nav.distance_to_home_m, "distance to home")
        check_num(nav.remaining_route_distance_m, "remaining route distance")
        check_num(nav.completed_route_distance_m, "completed route distance")
        check_num(nav.mission_progress_pct, "mission progress pct")

        # Check reliability numeric fields if present
        if state.reliability is not None:
            rel = state.reliability
            check_num(rel.mission_reliability_index, "reliability index")
            check_num(rel.estimated_rtb_time_s, "estimated RTB time")
            check_num(rel.estimated_rtb_energy_wh, "estimated RTB energy")
            check_num(rel.estimated_rtb_fuel_liters, "estimated RTB fuel")
            check_num(rel.available_energy_margin_pct, "available energy margin pct")
            check_num(rel.available_fuel_liters, "available fuel liters")
            check_num(rel.available_battery_wh, "available battery Wh")

        if state.decision is not None:
            dec = state.decision
            check_num(dec.decision_confidence, "decision confidence")
            check_num(dec.decision_timestamp, "decision timestamp")

        if state.trajectory is not None:
            traj = state.trajectory
            check_num(traj.timestamp_s, "trajectory timestamp")
            check_num(traj.route_distance_km, "trajectory route distance km")
            check_num(traj.estimated_route_time_min, "trajectory estimated time min")
            check_num(traj.estimated_route_fuel_l, "trajectory estimated fuel l")
            check_num(traj.estimated_route_battery_kwh, "trajectory estimated battery kwh")
            check_num(traj.route_optimization_score, "trajectory route optimization score")

        # -------------------------------------------------------------
        # 2. Timestamp Synchronization & Stale Data Detection
        # -------------------------------------------------------------
        subsystem_timestamps = {
            "engine": state.engine.timestamp_s,
            "battery": state.battery.timestamp_s,
            "electric_machine": state.electric_machine.timestamp_s,
            "powertrain": state.powertrain.timestamp_s,
            "environment": state.environment.timestamp_s,
            "mission": state.mission.timestamp_s,
        }
        if state.reliability is not None:
            subsystem_timestamps["reliability"] = state.reliability.timestamp_s
        if state.decision is not None:
            subsystem_timestamps["decision"] = state.decision.decision_timestamp

        is_any_stale = False
        for sub_name, sub_t in subsystem_timestamps.items():
            dt = abs(master_t - sub_t)
            if dt > stale_timeout_s:
                state.stale_data_flags[sub_name] = True
                warnings.append(f"Stale data detected in {sub_name} subsystem (lags by {dt:.3f}s)")
                is_any_stale = True
            else:
                state.stale_data_flags[sub_name] = False

        # Trajectory optimizer runs at 1.0s interval; tolerate up to 2.0s
        if state.trajectory is not None:
            traj_dt = abs(master_t - state.trajectory.timestamp_s)
            if traj_dt > 2.0:
                state.stale_data_flags["trajectory"] = True
                warnings.append(f"Stale trajectory optimization data (lags by {traj_dt:.3f}s)")
                is_any_stale = True
            else:
                state.stale_data_flags["trajectory"] = False

        # Predictive fast-forward simulation runs at 5.0s interval; tolerate up to 10.0s
        if state.prediction is not None:
            pred_dt = abs(master_t - state.prediction.prediction_timestamp)
            if pred_dt > 10.0:
                state.stale_data_flags["prediction"] = True
                warnings.append(f"Stale predictive simulation data (lags by {pred_dt:.3f}s)")
                is_any_stale = True
            else:
                state.stale_data_flags["prediction"] = False

        if is_any_stale:
            quality = DigitalTwinQualityStatus.DEGRADED

        # -------------------------------------------------------------
        # 3. Battery Boundary Checks
        # -------------------------------------------------------------
        soc = state.battery.soc_pct
        soh = state.battery.soh_pct
        energy_wh = state.battery.usable_energy_wh

        if not (0.0 <= soc <= 100.0):
            errors.append(f"Battery SoC out of physical bounds [0, 100]: {soc:.2f}%")
        if not (0.0 <= soh <= 100.0):
            errors.append(f"Battery SoH out of physical bounds [0, 100]: {soh:.2f}%")
        if energy_wh < -0.01:
            errors.append(f"Battery usable energy cannot be negative: {energy_wh:.2f} Wh")

        # -------------------------------------------------------------
        # 4. Cross-System Power Balance Conservation Check
        # -------------------------------------------------------------
        p_act = state.powertrain.propulsion_power_actual_w
        p_eng = state.powertrain.engine_power_contribution_w
        p_mot = state.powertrain.electric_power_contribution_w
        p_sum = p_eng + p_mot

        power_balance_diff = abs(p_act - p_sum)
        if power_balance_diff > 50.0:
            errors.append(
                f"Power conservation violation: P_actual ({p_act:.1f}W) != P_eng ({p_eng:.1f}W) + P_mot ({p_mot:.1f}W). Diff = {power_balance_diff:.1f}W"
            )
        elif power_balance_diff > 1.0:
            warnings.append(f"Minor power balance discrepancy: {power_balance_diff:.2f} W")

        # -------------------------------------------------------------
        # 5. Power Deficit Verification
        # -------------------------------------------------------------
        p_req = state.powertrain.propulsion_power_req_w
        expected_deficit = max(0.0, p_req - p_act)
        reported_deficit = state.powertrain.propulsion_power_deficit_w
        deficit_diff = abs(reported_deficit - expected_deficit)

        if deficit_diff > 50.0:
            errors.append(
                f"Power deficit mismatch: Reported ({reported_deficit:.1f}W) vs Expected ({expected_deficit:.1f}W)"
            )
        elif deficit_diff > 1.0:
            warnings.append(f"Minor power deficit mismatch: {deficit_diff:.2f} W")

        # -------------------------------------------------------------
        # 6. Navigation Geographic Boundary & Consistency Checks
        # -------------------------------------------------------------
        lat = nav.current_position.latitude_deg
        lon = nav.current_position.longitude_deg
        alt = nav.current_position.altitude_m

        if not (-90.0 <= lat <= 90.0):
            errors.append(f"Latitude out of physical geographic bounds [-90, 90]: {lat:.4f}°")
        if not (-180.0 <= lon <= 180.0):
            errors.append(f"Longitude out of physical geographic bounds [-180, 180]: {lon:.4f}°")
        if not (-100.0 <= alt <= 25000.0):
            errors.append(f"Altitude out of simulation bounds [-100m, 25000m]: {alt:.1f}m")

        if nav.distance_to_next_waypoint_m < -0.01:
            errors.append(f"Distance to next waypoint cannot be negative: {nav.distance_to_next_waypoint_m:.1f}m")
        if nav.distance_to_home_m < -0.01:
            errors.append(f"Distance to home cannot be negative: {nav.distance_to_home_m:.1f}m")
        if nav.remaining_route_distance_m < -0.01:
            errors.append(f"Remaining route distance cannot be negative: {nav.remaining_route_distance_m:.1f}m")
        if nav.completed_route_distance_m < -0.01:
            errors.append(f"Completed route distance cannot be negative: {nav.completed_route_distance_m:.1f}m")

        if not (0.0 <= nav.mission_progress_pct <= 100.0):
            errors.append(f"Mission progress % out of range [0, 100]: {nav.mission_progress_pct:.2f}%")

        if nav.total_waypoint_count > 0:
            if not (0 <= nav.current_waypoint_index < nav.total_waypoint_count):
                errors.append(
                    f"Current waypoint index ({nav.current_waypoint_index}) invalid for waypoint count ({nav.total_waypoint_count})"
                )

        if nav.completed_route_distance_m > (nav.total_route_distance_m + 1.0):
            errors.append(
                f"Completed route distance ({nav.completed_route_distance_m:.1f}m) exceeds total route distance ({nav.total_route_distance_m:.1f}m)"
            )

        # -------------------------------------------------------------
        # 7. Mission Reliability & Contingency Consistency Checks
        # -------------------------------------------------------------
        if state.reliability is not None:
            rel = state.reliability
            if not (0.0 <= rel.mission_reliability_index <= 100.0):
                errors.append(
                    f"Mission reliability index out of bounds [0, 100]: {rel.mission_reliability_index:.2f}"
                )
            if rel.estimated_rtb_time_s < -0.01:
                errors.append(f"Estimated RTB time cannot be negative: {rel.estimated_rtb_time_s:.1f}s")
            if rel.estimated_rtb_energy_wh < -0.01:
                errors.append(f"Estimated RTB energy cannot be negative: {rel.estimated_rtb_energy_wh:.1f}Wh")
            if rel.estimated_rtb_fuel_liters < -0.01:
                errors.append(f"Estimated RTB fuel cannot be negative: {rel.estimated_rtb_fuel_liters:.2f}L")

        # -------------------------------------------------------------
        # 8. Decision & Trajectory Optimization Consistency Checks
        # -------------------------------------------------------------
        if state.decision is not None:
            dec = state.decision
            if dec.recommended_action not in [
                "CONTINUE_MISSION", "ABORT_AND_RTB", "DIVERT_TO_ALTERNATE", "EMERGENCY_DESCENT"
            ]:
                errors.append(f"Invalid decision recommended_action: {dec.recommended_action}")
            if not (0.0 <= dec.decision_confidence <= 1.0):
                errors.append(f"Decision confidence out of bounds [0, 1]: {dec.decision_confidence}")

        if state.trajectory is not None:
            traj = state.trajectory
            if traj.route_distance_km < -0.01:
                errors.append(f"Trajectory distance cannot be negative: {traj.route_distance_km}")
            if traj.estimated_route_time_min < -0.01:
                errors.append(f"Trajectory time cannot be negative: {traj.estimated_route_time_min}")
            if not (0.0 <= traj.route_optimization_score <= 100.0):
                errors.append(f"Trajectory score out of bounds [0, 100]: {traj.route_optimization_score}")

        # -------------------------------------------------------------
        # 8b. Mission Re-routing Consistency Checks
        # -------------------------------------------------------------
        if state.rerouting is not None:
            rr = state.rerouting
            if rr.pending_proposal_count < 0:
                errors.append(f"Invalid pending proposal count: {rr.pending_proposal_count}")
            valid_prop_statuses = [
                "NO_PROPOSAL", "PENDING_OPERATOR_REVIEW", "APPROVED", "REJECTED",
                "EXPIRED", "SUPERSEDED", "APPLIED", "FAILED"
            ]
            if rr.proposal_status not in valid_prop_statuses:
                errors.append(f"Invalid proposal status: {rr.proposal_status}")

        # -------------------------------------------------------------
        # 9. Evaluate Final Quality Status
        # -------------------------------------------------------------
        state.validation_warnings = warnings
        state.validation_errors = errors

        if len(errors) > 0:
            state.quality_status = DigitalTwinQualityStatus.INVALID
        elif is_any_stale:
            state.quality_status = DigitalTwinQualityStatus.DEGRADED
        elif len(warnings) > 0:
            state.quality_status = DigitalTwinQualityStatus.WARNING
        else:
            state.quality_status = DigitalTwinQualityStatus.HEALTHY

        return state.quality_status


class UAVDigitalTwinHistoryBuffer:
    """
    Lightweight bounded chronological history buffer storing recent Digital Twin Telemetry Snapshots.
    Maintains fixed max length with zero unbounded memory growth.
    """

    def __init__(self, max_length: int = 500):
        self.max_length = max(10, int(max_length))
        self._buffer: deque = deque(maxlen=self.max_length)

    def append(self, snapshot: UAVDigitalTwinTelemetry):
        """Appends a new telemetry snapshot in chronological order."""
        self._buffer.append(snapshot)

    def get_latest(self) -> Optional[UAVDigitalTwinTelemetry]:
        """Returns the most recent Digital Twin snapshot."""
        if len(self._buffer) == 0:
            return None
        return self._buffer[-1]

    def get_history(self, count: Optional[int] = None) -> List[UAVDigitalTwinTelemetry]:
        """Returns list of recent snapshots in chronological order."""
        if count is None or count >= len(self._buffer):
            return list(self._buffer)
        return list(self._buffer)[-count:]

    def clear(self):
        """Clears buffer contents."""
        self._buffer.clear()

    def __len__(self) -> int:
        return len(self._buffer)


class UAVDigitalTwin:
    """
    Orchestration and Synchronization Layer for the Unified UAV Digital Twin.
    Consumes physical subsystem outputs, synchronizes state, runs cross-system validation,
    and emits immutable telemetry snapshots into a bounded history buffer.
    """

    def __init__(self, history_buffer_size: int = 500, stale_timeout_s: float = 0.20):
        self.history_buffer = UAVDigitalTwinHistoryBuffer(max_length=history_buffer_size)
        self.stale_timeout_s = stale_timeout_s
        self.navigation_engine = SpatialNavigationEngine()
        self.reliability_engine = MissionReliabilityEngine()
        self.decision_engine = MissionDecisionEngine()
        self.trajectory_optimizer = TrajectoryOptimizer()
        self.predictive_analyzer = PredictiveRiskAnalyzer()
        self.last_prediction_time_s: Optional[float] = None
        self.cached_prediction: Optional[PredictiveSimulationTelemetry] = None
        self.last_predictive_report: Optional[PredictiveRiskReport] = None
        self.rerouting_manager = MissionReroutingManager()
        self.prediction_interval_s: float = 5.0
        self.latest_state: Optional[UAVDigitalTwinState] = None

    def reset(self):
        """Resets Digital Twin state, navigation engine, reliability engine, decision engine, trajectory optimizer, and history buffer."""
        self.history_buffer.clear()
        self.navigation_engine.reset()
        self.reliability_engine.reset()
        self.decision_engine = MissionDecisionEngine()
        self.trajectory_optimizer = TrajectoryOptimizer()
        self.predictive_analyzer = PredictiveRiskAnalyzer()
        self.last_prediction_time_s = None
        self.cached_prediction = None
        self.last_predictive_report = None
        self.rerouting_manager = MissionReroutingManager()
        self.latest_state = None

    def update_from_hybrid_telemetry(
        self,
        hybrid_telemetry: HybridPowertrainTelemetry,
        flight_state: FlightState,
        current_position: Optional[GeographicPosition] = None,
        engine_health_pct: float = 100.0,
        dt_s: float = 0.05
    ) -> UAVDigitalTwinTelemetry:
        """
        Synchronizes Digital Twin State from a HybridPowertrainTelemetry snapshot and FlightState.
        """
        t = hybrid_telemetry.timestamp_s

        # Interpolate/Estimate UAV Geographic Position if not explicitly provided
        if current_position is None:
            wps = self.navigation_engine.waypoints
            if len(wps) >= 2:
                # Estimate position along route based on simulation time
                tot_dist = self.navigation_engine.total_planned_route_distance_m
                dist_traveled = (t * flight_state.airspeed_mps) % max(1.0, tot_dist)
                
                # Interpolate along route
                curr_dist = 0.0
                target_lat, target_lon, target_alt = wps[0].latitude_deg, wps[0].longitude_deg, flight_state.altitude_m
                for i in range(len(wps) - 1):
                    seg_len = haversine_distance_m(wps[i].to_position(), wps[i+1].to_position())
                    if curr_dist + seg_len >= dist_traveled:
                        frac = (dist_traveled - curr_dist) / max(0.1, seg_len)
                        target_lat = wps[i].latitude_deg + frac * (wps[i+1].latitude_deg - wps[i].latitude_deg)
                        target_lon = wps[i].longitude_deg + frac * (wps[i+1].longitude_deg - wps[i].longitude_deg)
                        target_alt = flight_state.altitude_m
                        break
                    curr_dist += seg_len
                
                current_position = GeographicPosition(target_lat, target_lon, target_alt, timestamp_s=t)
            else:
                current_position = GeographicPosition(18.5822, 73.9197, flight_state.altitude_m, timestamp_s=t)

        # 1. Update Spatial Navigation Engine
        nav_state: MissionNavigationState = self.navigation_engine.update(
            current_position=current_position,
            ground_speed_mps=flight_state.airspeed_mps,
            timestamp_s=t
        )

        # 2. Environment Subsystem State
        env_state = EnvironmentSubsystemState(
            timestamp_s=t,
            altitude_ft=flight_state.altitude_ft,
            altitude_m=flight_state.altitude_m,
            ambient_temp_c=flight_state.ambient_temp_c,
            ambient_pressure_bar=flight_state.ambient_pressure_bar,
            air_density_kgpm3=flight_state.air_density_kgpm3,
            airspeed_mps=flight_state.airspeed_mps,
            flight_phase=flight_state.phase
        )

        # 3. Engine Subsystem State
        eng_state = EngineSubsystemState(
            timestamp_s=t,
            rpm=hybrid_telemetry.engine_rpm,
            power_hp=hybrid_telemetry.engine_power_contribution_hp,
            power_w=hybrid_telemetry.engine_power_contribution_w,
            throttle_pct=hybrid_telemetry.engine_throttle_pct,
            manifold_pressure_bar=2.45 if hybrid_telemetry.engine_throttle_pct > 50 else 1.02,
            oil_pressure_bar=4.5,
            oil_temp_c=90.0,
            coolant_temp_c=85.0,
            fuel_flow_lph=hybrid_telemetry.engine_fuel_flow_lph,
            egt_c=[750.0] * 4,
            cht_c=[175.0] * 4,
            vibration_rms_g=1.2
        )

        # 4. Battery Subsystem State
        batt_state = BatterySubsystemState(
            timestamp_s=t,
            soc_pct=hybrid_telemetry.battery_soc_pct,
            soh_pct=hybrid_telemetry.battery_soh_pct,
            terminal_voltage_v=hybrid_telemetry.battery_voltage_v,
            open_circuit_voltage_v=hybrid_telemetry.battery_voltage_v,
            internal_resistance_ohm=0.08,
            current_a=hybrid_telemetry.battery_current_a,
            power_demand_w=hybrid_telemetry.battery_power_w,
            power_actual_w=hybrid_telemetry.battery_power_w,
            power_loss_w=(hybrid_telemetry.battery_current_a ** 2) * 0.08,
            usable_energy_wh=hybrid_telemetry.battery_remaining_energy_kwh * 1000.0,
            usable_energy_kwh=hybrid_telemetry.battery_remaining_energy_kwh,
            temperature_c=hybrid_telemetry.battery_temperature_c,
            charge_state="CHARGING" if hybrid_telemetry.battery_current_a < -0.5 else ("DISCHARGING" if hybrid_telemetry.battery_current_a > 0.5 else "IDLE"),
            is_power_constrained=hybrid_telemetry.battery_is_constrained
        )

        # 5. Electric Machine Subsystem State
        em_state = ElectricMachineSubsystemState(
            timestamp_s=t,
            operating_mode=hybrid_telemetry.electric_machine_mode,
            mechanical_power_w=hybrid_telemetry.electric_machine_power_w,
            electrical_power_w=hybrid_telemetry.battery_power_w,
            torque_nm=hybrid_telemetry.electric_machine_torque_nm,
            rpm=hybrid_telemetry.engine_rpm,
            efficiency=hybrid_telemetry.electric_machine_efficiency,
            power_loss_w=abs(hybrid_telemetry.battery_power_w - hybrid_telemetry.electric_machine_power_w),
            temperature_c=hybrid_telemetry.electric_machine_temperature_c,
            is_thermal_warning=hybrid_telemetry.electric_machine_temperature_c >= 110.0
        )

        # 6. Hybrid Powertrain Subsystem State
        pt_state = HybridPowertrainSubsystemState(
            timestamp_s=t,
            operating_mode=hybrid_telemetry.operating_mode,
            control_mode=hybrid_telemetry.control_mode,
            propulsion_power_req_w=hybrid_telemetry.propulsion_power_req_w,
            propulsion_power_actual_w=hybrid_telemetry.propulsion_power_actual_w,
            propulsion_power_deficit_w=hybrid_telemetry.propulsion_power_deficit_w,
            engine_power_contribution_w=hybrid_telemetry.engine_power_contribution_w,
            electric_power_contribution_w=hybrid_telemetry.motor_power_contribution_w,
            energy_flow_direction=hybrid_telemetry.energy_flow_direction,
            is_system_power_constrained=hybrid_telemetry.is_system_power_constrained
        )

        # Construct unified state with preliminary subsystems
        uav_state = UAVDigitalTwinState(
            timestamp_s=t,
            environment=env_state,
            engine=eng_state,
            battery=batt_state,
            electric_machine=em_state,
            powertrain=pt_state,
            mission=nav_state
        )

        # Preliminary validation to determine data quality status
        prelim_quality = UAVDigitalTwinValidator.validate(uav_state, stale_timeout_s=self.stale_timeout_s)

        # 7. Evaluate Mission Reliability & Dynamic Contingency
        home_pos = self.navigation_engine.home_position
        rel_state = self.reliability_engine.evaluate(
            timestamp_s=t,
            current_pos=nav_state.current_position,
            home_pos=home_pos,
            altitude_m=flight_state.altitude_m,
            ground_speed_mps=flight_state.airspeed_mps,
            engine_health_pct=engine_health_pct,
            engine_power_avail_w=hybrid_telemetry.engine_power_contribution_w if hybrid_telemetry.operating_mode != "ELECTRIC_ONLY" else 149000.0,
            fuel_flow_lph=hybrid_telemetry.engine_fuel_flow_lph,
            battery_energy_wh=hybrid_telemetry.battery_remaining_energy_kwh * 1000.0,
            battery_soc_pct=hybrid_telemetry.battery_soc_pct,
            battery_soh_pct=hybrid_telemetry.battery_soh_pct,
            battery_temp_c=hybrid_telemetry.battery_temperature_c,
            battery_is_constrained=hybrid_telemetry.battery_is_constrained,
            electric_motor_power_avail_w=40000.0,
            current_power_demand_w=hybrid_telemetry.propulsion_power_req_w,
            power_deficit_w=hybrid_telemetry.propulsion_power_deficit_w,
            mission_progress_pct=nav_state.mission_progress_pct,
            quality_status=prelim_quality.value,
            stale_data_flags=uav_state.stale_data_flags,
            dt_s=dt_s
        )
        uav_state.reliability = rel_state

        # 8. Evaluate AI-Guided Mission Decision
        active_wp_name = nav_state.current_waypoint.name if nav_state.current_waypoint else "Target Waypoint"
        rem_fuel = self.reliability_engine.remaining_fuel_liters

        decision_res = self.decision_engine.evaluate(
            timestamp_s=t,
            quality_status=prelim_quality.value,
            engine_health_pct=engine_health_pct,
            battery_soc_pct=hybrid_telemetry.battery_soc_pct,
            battery_usable_kwh=hybrid_telemetry.battery_remaining_energy_kwh,
            battery_temp_c=hybrid_telemetry.battery_temperature_c,
            battery_is_constrained=hybrid_telemetry.battery_is_constrained,
            remaining_fuel_liters=rem_fuel,
            power_deficit_w=hybrid_telemetry.propulsion_power_deficit_w,
            propulsion_power_req_w=hybrid_telemetry.propulsion_power_req_w,
            distance_to_home_m=nav_state.distance_to_home_m,
            remaining_route_distance_m=nav_state.remaining_route_distance_m,
            reliability_state=rel_state,
            stale_data_flags=uav_state.stale_data_flags,
            next_waypoint_name=active_wp_name,
            home_destination_name="Pune Lohegaon (Home Base)"
        )
        uav_state.decision = decision_res

        # 9. Evaluate Multi-Criteria Trajectory Optimization
        traj_telemetry = self.trajectory_optimizer.optimize_trajectory(
            current_pos=nav_state.current_position,
            home_pos=home_pos,
            active_waypoint_index=nav_state.current_waypoint_index,
            all_mission_waypoints=self.navigation_engine.waypoints,
            remaining_fuel_liters=rem_fuel,
            battery_soc_pct=hybrid_telemetry.battery_soc_pct,
            engine_health_pct=engine_health_pct,
            ground_speed_mps=flight_state.airspeed_mps,
            timestamp_s=t
        )
        uav_state.trajectory = traj_telemetry

        # 10. Evaluate Multi-Scenario Predictive Fast-Forward Simulation (Throttled at 5.0s interval)
        candidates = self.trajectory_optimizer.last_candidate_routes
        if candidates and (self.last_prediction_time_s is None or (t - self.last_prediction_time_s) >= self.prediction_interval_s):
            pred_report = self.predictive_analyzer.compare_candidate_routes(
                candidate_routes=candidates,
                initial_fuel_liters=rem_fuel,
                initial_soc_pct=hybrid_telemetry.battery_soc_pct,
                initial_soh_pct=hybrid_telemetry.battery_soh_pct,
                initial_battery_temp_c=hybrid_telemetry.battery_temperature_c,
                initial_engine_cht_c=eng_state.cht_c[0] if eng_state.cht_c else 175.0,
                initial_engine_oil_t_c=eng_state.oil_temp_c,
                initial_motor_temp_c=em_state.temperature_c,
                timestamp_s=t
            )
            self.last_predictive_report = pred_report
            self.cached_prediction = self.predictive_analyzer.generate_telemetry_snapshot(pred_report)
            self.last_prediction_time_s = t

        uav_state.prediction = self.cached_prediction

        # 11. Evaluate Closed-Loop Mission Re-Routing & Operator Confirmation
        reroute_telem = self.rerouting_manager.evaluate_and_update(
            twin_state=uav_state,
            candidate_routes=candidates,
            predictive_report=self.last_predictive_report
        )
        uav_state.rerouting = reroute_telem

        # Final validation pass including reliability, decision, trajectory, prediction, and rerouting
        UAVDigitalTwinValidator.validate(uav_state, stale_timeout_s=self.stale_timeout_s)

        self.latest_state = uav_state

        pred = uav_state.prediction

        # Emit telemetry snapshot
        snapshot = UAVDigitalTwinTelemetry(
            timestamp_s=uav_state.timestamp_s,
            quality_status=uav_state.quality_status.value,
            environment=uav_state.environment,
            engine=uav_state.engine,
            battery=uav_state.battery,
            electric_machine=uav_state.electric_machine,
            powertrain=uav_state.powertrain,
            mission=uav_state.mission,
            reliability=uav_state.reliability,
            decision=uav_state.decision,
            trajectory=uav_state.trajectory,
            prediction=pred,
            prediction_status=pred.prediction_status if pred else "UNAVAILABLE",
            prediction_timestamp=pred.prediction_timestamp if pred else None,
            prediction_scenario_count=pred.prediction_scenario_count if pred else 0,
            predicted_mission_completion_rate=pred.predicted_mission_completion_rate if pred else None,
            predicted_rtb_feasibility_rate=pred.predicted_rtb_feasibility_rate if pred else None,
            predicted_battery_depletion_rate=pred.predicted_battery_depletion_rate if pred else None,
            predicted_engine_thermal_risk=pred.predicted_engine_thermal_risk if pred else None,
            predicted_battery_thermal_risk=pred.predicted_battery_thermal_risk if pred else None,
            predicted_fuel_reserve_violation_rate=pred.predicted_fuel_reserve_violation_rate if pred else None,
            predicted_energy_reserve_p05=pred.predicted_energy_reserve_p05 if pred else None,
            predicted_energy_reserve_p95=pred.predicted_energy_reserve_p95 if pred else None,
            recommended_predictive_route=pred.recommended_predictive_route if pred else None,
            prediction_computation_time_ms=pred.prediction_computation_time_ms if pred else None,
            rerouting=uav_state.rerouting,
            reroute_status=reroute_telem.rerouting_subsystem_status if reroute_telem else "NOMINAL",
            reroute_pending_count=reroute_telem.pending_proposal_count if reroute_telem else 0,
            reroute_active_proposal_id=reroute_telem.active_proposal_id if reroute_telem else None,
            reroute_proposal_status=reroute_telem.proposal_status if reroute_telem else None,
            reroute_active_route_id=reroute_telem.active_route_id if reroute_telem else "ORIGINAL_MISSION_ROUTE",
            reroute_proposed_route_id=reroute_telem.proposed_route_id if reroute_telem else None,
            reroute_proposed_destination=reroute_telem.proposed_destination if reroute_telem else None,
            reroute_contingency_detected=reroute_telem.contingency_detected if reroute_telem else False,
            reroute_attention_required=reroute_telem.operator_attention_required if reroute_telem else False,
            reroute_last_operator_action=reroute_telem.last_operator_action if reroute_telem else "NONE",
            validation_warnings=uav_state.validation_warnings,
            validation_errors=uav_state.validation_errors,
            stale_data_flags=uav_state.stale_data_flags,
            provenance="SIMULATED"
        )

        # Store in history buffer
        self.history_buffer.append(snapshot)
        return snapshot
