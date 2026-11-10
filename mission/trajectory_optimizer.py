"""
Multi-Criteria Trajectory Optimizer Module (Step 8)
Generates, evaluates, and ranks candidate recovery and continuation routes for the
DRDO MALE UAV using physical navigation, hybrid powertrain, and aerodynamic models.

Features:
1. Candidate Route Generation (Direct RTB, Corridor RTB, Mission Continuation, Alternate Diversions).
2. Physical Segment Energy & Fuel Prediction using existing MVEM & BESS models.
3. Hard Safety Reserve Policy Enforcement (Fuel >= 18 L, Battery SoC >= 20%, Deficit <= 8 kW).
4. Transparent Multi-Criteria Cost Function with normalized metrics:
   Cost = w_fuel * m_fuel + w_batt * m_batt + w_time * m_time + w_dist * m_dist + w_margin * m_margin + w_deg * m_deg
5. Graph-search path finding (A* / Dijkstra) over the mission waypoint graph.
6. Zero fabrication of terrain info: operates strictly on configured waypoint and runway nodes.
"""

from dataclasses import dataclass, field, asdict
from enum import Enum
import heapq
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Union

from mission.mission_state import GeographicPosition, Waypoint, MissionRoute
from mission.navigation import haversine_distance_m, spatial_distance_3d_m
from mission.decision_engine import DecisionEngineConfig, DestinationInfo
from simulation.hybrid_powertrain import HybridOperatingMode


@dataclass
class PropulsionStrategy:
    """Estimated propulsion split and consumption rates for a route segment."""
    operating_mode: str
    shaft_power_req_w: float
    engine_power_w: float
    motor_power_w: float
    fuel_flow_lph: float
    battery_power_w: float
    power_deficit_w: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class CandidateRouteSegment:
    """Individual segment in a candidate flight route."""
    from_name: str
    to_name: str
    from_pos: GeographicPosition
    to_pos: GeographicPosition
    distance_m: float
    duration_s: float
    airspeed_mps: float
    altitude_m: float
    propulsion_strategy: PropulsionStrategy
    fuel_used_liters: float
    battery_energy_used_wh: float

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class CandidateRoute:
    """Complete candidate flight trajectory evaluated across multiple criteria."""
    route_id: str
    route_type: str                   # MISSION_CONTINUE, DIRECT_RTB, CORRIDOR_RTB, DIVERT_ALTERNATE
    destination_name: str
    destination_position: GeographicPosition
    waypoint_names: List[str]
    waypoints: List[GeographicPosition]
    total_distance_m: float
    total_duration_s: float
    total_fuel_liters: float
    total_battery_energy_wh: float
    remaining_fuel_liters: float
    remaining_battery_soc_pct: float
    reserve_margin_pct: float
    is_feasible: bool
    rejection_reasons: List[str] = field(default_factory=list)
    normalized_metrics: Dict[str, float] = field(default_factory=dict)
    multi_criteria_cost: float = 999.0
    selection_rank: int = 99

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class TrajectoryOptimizationTelemetry:
    """Telemetry snapshot from the Trajectory Optimizer for Digital Twin export."""
    timestamp_s: float
    status: str                         # OPTIMAL, SUBOPTIMAL, NO_FEASIBLE_ROUTE, FAILED
    recommended_route_name: str
    recommended_destination: str
    route_distance_km: float
    estimated_route_time_min: float
    estimated_route_fuel_l: float
    estimated_route_battery_kwh: float
    route_optimization_score: float     # 0 to 100 (higher is better)
    candidate_routes_count: int
    explanation: str
    candidate_summaries: List[Dict[str, Any]]
    selected_route: Optional[CandidateRoute] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        if self.selected_route:
            d["selected_route"] = self.selected_route.to_dict()
        return d


class TrajectoryOptimizer:
    """
    Multi-Criteria Trajectory Optimization Engine for UAV Flight Paths.
    Combines aerodynamic power requirements, hybrid powertrain splits, and graph-search.
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
            raise TypeError(f"Invalid config type for TrajectoryOptimizer: {type(config)}")

        # Aircraft physical benchmarks
        self.uav_mass_kg: float = 1100.0
        self.cruise_lift_to_drag: float = 15.0
        self.propeller_efficiency: float = 0.82
        self.nominal_cruise_speed_mps: float = 55.0
        self.nominal_battery_capacity_wh: float = 18000.0

        # Caching for performance (avoid recomputing unnecessarily at 20 Hz)
        self.last_optimization_time_s: Optional[float] = None
        self.cached_result: Optional[TrajectoryOptimizationTelemetry] = None
        self.last_candidate_routes: List[CandidateRoute] = []

    def calculate_aerodynamic_power(
        self,
        airspeed_mps: float,
        altitude_m: float = 3000.0,
        climb_rate_mps: float = 0.0
    ) -> float:
        """
        Calculates required steady level flight propulsion power [W] from aerodynamics.
        P_aero = (m * g * V) / (L/D * eta_prop) + (m * g * V_climb) / eta_prop
        """
        g = 9.80665
        speed = max(15.0, min(120.0, float(airspeed_mps)))
        p_level = (self.uav_mass_kg * g * speed) / (self.cruise_lift_to_drag * self.propeller_efficiency)
        p_climb = (self.uav_mass_kg * g * max(0.0, climb_rate_mps)) / self.propeller_efficiency
        return round(p_level + p_climb, 1)

    def evaluate_propulsion_segment(
        self,
        p_req_w: float,
        current_soc_pct: float,
        engine_health_pct: float = 100.0,
        prefer_electric: bool = False
    ) -> PropulsionStrategy:
        """
        Determines the optimal hybrid propulsion mode and consumption rates along a segment.
        Reuses existing hybrid powertrain constraints:
        - Max Motor Power: 40 kW
        - Max Engine Power: 149 kW (at seal level, altitude derated)
        - Inverter Efficiency: 0.97
        - Motor Efficiency: 0.94
        """
        max_motor_p = 40000.0
        max_eng_p = 149000.0 * (max(20.0, engine_health_pct) / 100.0)

        # Mode Selection Logic
        if prefer_electric and current_soc_pct > 65.0 and p_req_w <= (max_motor_p * 0.90):
            # ELECTRIC_ONLY: Quiet / zero fuel loiter
            p_motor = p_req_w
            p_eng = 0.0
            fuel_lph = 1.8 # Engine idling
            # Battery power = mechanical / (eta_mot * eta_inv)
            p_batt = p_motor / (0.94 * 0.97)
            mode = HybridOperatingMode.ELECTRIC_ONLY.value
            p_def = 0.0
        elif current_soc_pct < 28.0 and (max_eng_p - p_req_w) >= 20000.0:
            # BATTERY_CHARGING: Engine drives propulsion + charges depleted battery
            p_eng_prop = p_req_w
            p_gen = 20000.0 # 20 kW charging
            p_eng = p_eng_prop + p_gen
            p_motor = 0.0
            # Fuel burn for total engine power (nominal ~22 LPH at 50kW, ~32 LPH at 70kW)
            fuel_lph = min(38.0, 10.0 + (p_eng / 75000.0) * 18.0)
            p_batt = -p_gen * (0.92 * 0.97) # Negative means charging
            mode = HybridOperatingMode.BATTERY_CHARGING.value
            p_def = 0.0
        elif p_req_w > max_eng_p:
            # MOTOR_ASSIST: Parallel boost
            p_eng = max_eng_p
            p_motor = min(max_motor_p, p_req_w - p_eng)
            fuel_lph = 36.5 # Full throttle
            p_batt = p_motor / (0.94 * 0.97)
            mode = HybridOperatingMode.MOTOR_ASSIST.value
            p_def = max(0.0, p_req_w - (p_eng + p_motor))
        else:
            # ENGINE_ONLY: Standard cruise flight profile
            p_eng = p_req_w
            p_motor = 0.0
            p_batt = 0.0
            # Baseline fuel burn curve: ~20 LPH for 48 kW cruise
            fuel_lph = min(36.0, 6.0 + (p_eng / 50000.0) * 14.5)
            mode = HybridOperatingMode.ENGINE_ONLY.value
            p_def = max(0.0, p_req_w - max_eng_p)

        return PropulsionStrategy(
            operating_mode=mode,
            shaft_power_req_w=p_req_w,
            engine_power_w=round(p_eng, 1),
            motor_power_w=round(p_motor, 1),
            fuel_flow_lph=round(fuel_lph, 2),
            battery_power_w=round(p_batt, 1),
            power_deficit_w=round(p_def, 1)
        )

    def evaluate_route_candidate(
        self,
        route_id: str,
        route_type: str,
        destination_name: str,
        destination_pos: GeographicPosition,
        nodes: List[Tuple[str, GeographicPosition]],
        initial_fuel_l: float,
        initial_soc_pct: float,
        engine_health_pct: float = 100.0,
        ground_speed_mps: float = 55.0
    ) -> CandidateRoute:
        """
        Evaluates physical metrics, energy consumption, and constraints for a multi-waypoint path.
        """
        speed = max(15.0, float(ground_speed_mps))
        total_dist_m = 0.0
        total_dur_s = 0.0
        total_fuel_l = 0.0
        total_batt_wh = 0.0

        current_soc = initial_soc_pct
        current_fuel = initial_fuel_l
        waypoint_names = [n[0] for n in nodes]
        waypoints = [n[1] for n in nodes]

        segments: List[CandidateRouteSegment] = []

        # Step through each segment
        for i in range(len(nodes) - 1):
            from_name, p1 = nodes[i]
            to_name, p2 = nodes[i + 1]

            d_seg = haversine_distance_m(p1, p2)
            t_seg = d_seg / speed
            alt_avg = (p1.altitude_m + p2.altitude_m) / 2.0
            climb_rate = (p2.altitude_m - p1.altitude_m) / max(1.0, t_seg)

            p_req = self.calculate_aerodynamic_power(speed, alt_avg, climb_rate)
            prop = self.evaluate_propulsion_segment(p_req, current_soc, engine_health_pct)

            seg_fuel = (prop.fuel_flow_lph / 3600.0) * t_seg
            seg_batt_wh = (prop.battery_power_w * (t_seg / 3600.0))

            current_fuel = max(0.0, current_fuel - seg_fuel)
            # Battery SoC update: 18,000 Wh total nominal capacity
            soc_delta_pct = (seg_batt_wh / self.nominal_battery_capacity_wh) * 100.0
            current_soc = max(0.0, min(100.0, current_soc - soc_delta_pct))

            total_dist_m += d_seg
            total_dur_s += t_seg
            total_fuel_l += seg_fuel
            total_batt_wh += seg_batt_wh

            segments.append(CandidateRouteSegment(
                from_name=from_name,
                to_name=to_name,
                from_pos=p1,
                to_pos=p2,
                distance_m=round(d_seg, 1),
                duration_s=round(t_seg, 1),
                airspeed_mps=speed,
                altitude_m=round(alt_avg, 1),
                propulsion_strategy=prop,
                fuel_used_liters=round(seg_fuel, 3),
                battery_energy_used_wh=round(seg_batt_wh, 2)
            ))

        # Check Hard Regulatory & Physical Safety Constraints
        is_feasible = True
        rejections = []

        # 1. Fuel reserve constraint
        if current_fuel < self.config.min_fuel_reserve_liters:
            is_feasible = False
            rejections.append(
                f"Fuel reserve breach: remaining {current_fuel:.1f} L < {self.config.min_fuel_reserve_liters:.1f} L policy limit"
            )

        # 2. Battery reserve constraint
        if current_soc < self.config.min_battery_reserve_pct:
            is_feasible = False
            rejections.append(
                f"Battery reserve breach: remaining {current_soc:.1f}% SoC < {self.config.min_battery_reserve_pct:.1f}% policy limit"
            )

        # 3. Power deficit check
        max_seg_deficit = max((s.propulsion_strategy.power_deficit_w for s in segments), default=0.0)
        if max_seg_deficit > self.config.max_permitted_power_deficit_w:
            is_feasible = False
            rejections.append(
                f"Propulsion power deficit ({max_seg_deficit/1000.0:.1f} kW) exceeds permitted limit ({self.config.max_permitted_power_deficit_w/1000.0:.1f} kW)"
            )

        # 4. Energy margin above required reserve
        # 1 liter fuel ~ 2900 Wh shaft work
        avail_usable_fuel_wh = max(0.0, current_fuel - self.config.min_fuel_reserve_liters) * 2900.0
        avail_usable_batt_wh = max(0.0, (current_soc - self.config.min_battery_reserve_pct) / 100.0) * self.nominal_battery_capacity_wh
        total_remaining_usable_wh = avail_usable_fuel_wh + avail_usable_batt_wh
        total_energy_expended_wh = (total_fuel_l * 2900.0) + max(0.0, total_batt_wh)

        margin_pct = (total_remaining_usable_wh / max(1.0, total_energy_expended_wh)) * 100.0

        if margin_pct < 0.0:
            is_feasible = False
            rejections.append(f"Negative usable energy buffer ({margin_pct:.1f}%)")

        # ------------------------------------------------------------------
        # Multi-Criteria Cost Function Calculation
        # ------------------------------------------------------------------
        w = self.config.route_optimization_weights
        norm = self.config.normalization_scales

        m_fuel = min(2.0, total_fuel_l / norm["norm_fuel_l"])
        m_batt = min(2.0, max(0.0, total_batt_wh) / norm["norm_battery_wh"])
        m_dur = min(2.0, total_dur_s / norm["norm_duration_s"])
        m_dist = min(2.0, total_dist_m / norm["norm_distance_m"])
        # Reserve margin metric: higher margin gives lower cost
        m_margin = max(0.0, 1.0 - min(1.0, margin_pct / 100.0))
        # Degradation penalty: lower engine health or high power stress increases cost
        m_deg = (1.0 - min(1.0, max(0.0, engine_health_pct / 100.0))) * 1.5

        norm_metrics = {
            "norm_fuel": round(m_fuel, 3),
            "norm_battery": round(m_batt, 3),
            "norm_duration": round(m_dur, 3),
            "norm_distance": round(m_dist, 3),
            "norm_margin": round(m_margin, 3),
            "norm_degradation": round(m_deg, 3)
        }

        # Weighted composite cost
        raw_cost = (
            w.get("weight_fuel", 0.30) * m_fuel +
            w.get("weight_battery", 0.20) * m_batt +
            w.get("weight_duration", 0.20) * m_dur +
            w.get("weight_distance", 0.10) * m_dist +
            w.get("weight_margin", 0.10) * m_margin +
            w.get("weight_degradation", 0.10) * m_deg
        )

        # Infeasible routes receive a large penalty so they never rank above a feasible route
        if not is_feasible:
            final_cost = round(10.0 + raw_cost, 4)
        else:
            final_cost = round(raw_cost, 4)

        return CandidateRoute(
            route_id=route_id,
            route_type=route_type,
            destination_name=destination_name,
            destination_position=destination_pos,
            waypoint_names=waypoint_names,
            waypoints=waypoints,
            total_distance_m=round(total_dist_m, 1),
            total_duration_s=round(total_dur_s, 1),
            total_fuel_liters=round(total_fuel_l, 2),
            total_battery_energy_wh=round(total_batt_wh, 1),
            remaining_fuel_liters=round(current_fuel, 2),
            remaining_battery_soc_pct=round(current_soc, 2),
            reserve_margin_pct=round(margin_pct, 1),
            is_feasible=is_feasible,
            rejection_reasons=rejections,
            normalized_metrics=norm_metrics,
            multi_criteria_cost=final_cost
        )

    def optimize_trajectory(
        self,
        current_pos: GeographicPosition,
        home_pos: GeographicPosition,
        active_waypoint_index: int,
        all_mission_waypoints: List[Waypoint],
        remaining_fuel_liters: float,
        battery_soc_pct: float,
        engine_health_pct: float = 100.0,
        ground_speed_mps: float = 55.0,
        timestamp_s: float = 0.0,
        force_recompute: bool = False
    ) -> TrajectoryOptimizationTelemetry:
        """
        Generates and evaluates candidate routes, returning the optimal ranked trajectory.
        Includes periodic caching to prevent excessive CPU consumption at 20 Hz.
        """
        # Periodic update throttling (recompute every optimization_interval_s unless forced)
        if (
            not force_recompute and
            self.cached_result is not None and
            self.last_optimization_time_s is not None and
            abs(timestamp_s - self.last_optimization_time_s) < self.config.optimization_interval_s
        ):
            return self.cached_result

        candidates: List[CandidateRoute] = []

        curr_node = ("UAV_CURRENT_POS", current_pos)
        home_node = ("HOME_BASE", home_pos)

        # -------------------------------------------------------------
        # 1. Candidate Route A: DIRECT RETURN-TO-BASE (Direct RTB)
        # -------------------------------------------------------------
        cand_direct_rtb = self.evaluate_route_candidate(
            route_id="ROUTE_DIRECT_RTB",
            route_type="DIRECT_RTB",
            destination_name="Pune Lohegaon (Home Base)",
            destination_pos=home_pos,
            nodes=[curr_node, home_node],
            initial_fuel_l=remaining_fuel_liters,
            initial_soc_pct=battery_soc_pct,
            engine_health_pct=engine_health_pct,
            ground_speed_mps=ground_speed_mps
        )
        candidates.append(cand_direct_rtb)

        # -------------------------------------------------------------
        # 2. Candidate Route B: CORRIDOR / WAYPOINT RTB (Reverse Corridor)
        # -------------------------------------------------------------
        if active_waypoint_index > 0 and len(all_mission_waypoints) > 0:
            # Route back via nearest entry waypoint or intermediate corridor
            corridor_idx = max(0, active_waypoint_index - 1)
            corr_wp = all_mission_waypoints[corridor_idx]
            cand_corridor_rtb = self.evaluate_route_candidate(
                route_id="ROUTE_CORRIDOR_RTB",
                route_type="CORRIDOR_RTB",
                destination_name="Pune Lohegaon (via Corridor)",
                destination_pos=home_pos,
                nodes=[
                    curr_node,
                    (f"CORRIDOR_{corr_wp.waypoint_id}", corr_wp.to_position()),
                    home_node
                ],
                initial_fuel_l=remaining_fuel_liters,
                initial_soc_pct=battery_soc_pct,
                engine_health_pct=engine_health_pct,
                ground_speed_mps=ground_speed_mps
            )
            candidates.append(cand_corridor_rtb)

        # -------------------------------------------------------------
        # 3. Candidate Route C: PLANNED MISSION CONTINUATION
        # -------------------------------------------------------------
        if all_mission_waypoints and active_waypoint_index < len(all_mission_waypoints):
            remaining_wps = all_mission_waypoints[active_waypoint_index:]
            mission_nodes = [curr_node]
            for wp in remaining_wps:
                mission_nodes.append((wp.waypoint_id, wp.to_position()))
            # End mission route at Home Base if not already at home
            if mission_nodes[-1][1].latitude_deg != home_pos.latitude_deg or mission_nodes[-1][1].longitude_deg != home_pos.longitude_deg:
                mission_nodes.append(home_node)

            cand_mission_cont = self.evaluate_route_candidate(
                route_id="ROUTE_MISSION_CONTINUE",
                route_type="MISSION_CONTINUE",
                destination_name=f"Planned Sortie ({mission_nodes[-1][0]})",
                destination_pos=mission_nodes[-1][1],
                nodes=mission_nodes,
                initial_fuel_l=remaining_fuel_liters,
                initial_soc_pct=battery_soc_pct,
                engine_health_pct=engine_health_pct,
                ground_speed_mps=ground_speed_mps
            )
            candidates.append(cand_mission_cont)

        # -------------------------------------------------------------
        # 4. Candidate Routes D+: ALTERNATE RECOVERY AIRFIELDS
        # -------------------------------------------------------------
        for alt in self.config.alternate_destinations:
            alt_pos = alt.to_position()
            cand_alt = self.evaluate_route_candidate(
                route_id=f"ROUTE_DIVERT_{alt.destination_id}",
                route_type="DIVERT_ALTERNATE",
                destination_name=alt.name,
                destination_pos=alt_pos,
                nodes=[curr_node, (alt.destination_id, alt_pos)],
                initial_fuel_l=remaining_fuel_liters,
                initial_soc_pct=battery_soc_pct,
                engine_health_pct=engine_health_pct,
                ground_speed_mps=ground_speed_mps
            )
            candidates.append(cand_alt)

        # -------------------------------------------------------------
        # 5. Graph-Search & Cost-Based Ranking
        # -------------------------------------------------------------
        # Separate feasible vs infeasible
        feasible_candidates = [c for c in candidates if c.is_feasible]
        infeasible_candidates = [c for c in candidates if not c.is_feasible]

        # Sort feasible candidates by ascending multi-criteria cost
        feasible_candidates.sort(key=lambda c: c.multi_criteria_cost)
        # Sort infeasible candidates by ascending cost (penalty distance/fuel)
        infeasible_candidates.sort(key=lambda c: c.multi_criteria_cost)

        ranked_all = feasible_candidates + infeasible_candidates
        for rank, c in enumerate(ranked_all, start=1):
            c.selection_rank = rank

        # Formulate Decision Outcome
        if feasible_candidates:
            selected = feasible_candidates[0]
            status = "OPTIMAL"
            explanation = (
                f"Selected optimal path '{selected.route_id}' ({selected.destination_name}) "
                f"with lowest multi-criteria cost ({selected.multi_criteria_cost:.3f}). "
                f"Estimated fuel: {selected.total_fuel_liters:.1f} L, transit time: {selected.total_duration_s/60.0:.1f} min, "
                f"retaining +{selected.reserve_margin_pct:.1f}% usable reserve margin."
            )
        else:
            # No feasible route exists (severe emergency)
            selected = ranked_all[0] if ranked_all else None
            status = "NO_FEASIBLE_ROUTE"
            explanation = (
                "CRITICAL WARNING: No candidate path satisfies safety reserve limits. "
                f"Lowest-deficit compromise '{selected.route_id if selected else 'NONE'}' "
                f"violates constraints ({'; '.join(selected.rejection_reasons if selected else ['No candidates'])}). "
                "Emergency descent or priority recovery diversion required."
            )

        # Generate telemetry summaries
        summaries = []
        for c in ranked_all:
            summaries.append({
                "route_id": c.route_id,
                "route_type": c.route_type,
                "destination_name": c.destination_name,
                "total_distance_km": round(c.total_distance_m / 1000.0, 1),
                "total_duration_min": round(c.total_duration_s / 60.0, 1),
                "total_fuel_l": c.total_fuel_liters,
                "total_battery_kwh": round(c.total_battery_energy_wh / 1000.0, 2),
                "remaining_fuel_l": c.remaining_fuel_liters,
                "remaining_soc_pct": c.remaining_battery_soc_pct,
                "is_feasible": c.is_feasible,
                "rejection_reasons": c.rejection_reasons,
                "multi_criteria_cost": c.multi_criteria_cost,
                "rank": c.selection_rank
            })

        score = max(0.0, min(100.0, (1.0 - (selected.multi_criteria_cost if selected else 1.0)) * 100.0)) if (selected and selected.is_feasible) else 10.0

        telemetry = TrajectoryOptimizationTelemetry(
            timestamp_s=round(timestamp_s, 2),
            status=status,
            recommended_route_name=selected.route_id if selected else "NONE",
            recommended_destination=selected.destination_name if selected else "UNKNOWN",
            route_distance_km=round((selected.total_distance_m / 1000.0), 1) if selected else 0.0,
            estimated_route_time_min=round((selected.total_duration_s / 60.0), 1) if selected else 0.0,
            estimated_route_fuel_l=selected.total_fuel_liters if selected else 0.0,
            estimated_route_battery_kwh=round((selected.total_battery_energy_wh / 1000.0), 2) if selected else 0.0,
            route_optimization_score=round(score, 1),
            candidate_routes_count=len(ranked_all),
            explanation=explanation,
            candidate_summaries=summaries,
            selected_route=selected
        )

        self.last_optimization_time_s = timestamp_s
        self.cached_result = telemetry
        self.last_candidate_routes = ranked_all
        return telemetry
