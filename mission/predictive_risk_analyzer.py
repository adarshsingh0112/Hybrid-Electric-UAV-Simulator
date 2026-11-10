"""
Predictive Risk Analyzer Module (Step 9)
Aggregates Monte Carlo fast-forward simulation runs into empirical risk metrics,
statistical uncertainty intervals (5th, 50th, 95th percentiles), and comparative
multi-route evaluations.

Important Aeronautical Disclaimers:
- All completion rates and risk factors represent empirical frequencies within the
  simulated Monte Carlo ensemble under defined physics and uncertainty models.
- These results do NOT constitute formal DO-178C or airworthiness certified probabilities.
"""

from dataclasses import dataclass, field, asdict
import math
from pathlib import Path
import time
from typing import Dict, List, Optional, Tuple, Any, Union

from simulation.scenario_generator import FastForwardConfig, ScenarioParameters, ScenarioGenerator
from simulation.fast_forward_engine import (
    FastForwardSimulationEngine,
    ScenarioSimulationRecord,
)
from mission.trajectory_optimizer import CandidateRoute


def compute_percentile(values: List[float], p: float) -> float:
    """Computes p-th percentile (0 <= p <= 100) using linear interpolation."""
    if not values:
        return 0.0
    sorted_v = sorted(values)
    k = (len(sorted_v) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_v[int(k)]
    d0 = sorted_v[int(f)] * (c - k)
    d1 = sorted_v[int(c)] * (k - f)
    return d0 + d1


def compute_wilson_score_interval(successes: int, total: int, z: float = 1.96) -> Tuple[float, float]:
    """
    Computes Wilson score confidence interval for a binomial proportion.
    Accurate for small sample sizes and extreme probabilities (unlike normal approximation).
    """
    if total <= 0:
        return (0.0, 0.0)
    p = successes / total
    denom = 1.0 + (z ** 2) / total
    center = (p + (z ** 2) / (2.0 * total)) / denom
    margin = (z * math.sqrt((p * (1.0 - p) + (z ** 2) / (4.0 * total)) / total)) / denom
    lower = max(0.0, center - margin) * 100.0
    upper = min(1.0, center + margin) * 100.0
    return (round(lower, 1), round(upper, 1))


@dataclass
class RouteRiskSummary:
    """
    Empirical statistical risk summary for a candidate flight route across simulated scenarios.
    """
    route_id: str
    route_type: str
    destination_name: str
    distance_km: float
    scenario_count: int

    # Completion and viability frequencies
    completion_rate_pct: float
    completion_ci_95_pct: Tuple[float, float]
    fuel_reserve_violation_rate_pct: float
    battery_reserve_violation_rate_pct: float
    battery_depletion_rate_pct: float
    thermal_risk_rate_pct: float
    power_deficit_rate_pct: float

    # Flight duration percentiles [minutes]
    duration_mean_min: float
    duration_std_min: float
    duration_p05_min: float
    duration_p50_min: float
    duration_p95_min: float

    # Fuel remaining percentiles [liters]
    fuel_remaining_p05_l: float
    fuel_remaining_p50_l: float
    fuel_remaining_p95_l: float
    fuel_consumed_mean_l: float

    # Battery SoC remaining percentiles [%]
    battery_soc_p05_pct: float
    battery_soc_p50_pct: float
    battery_soc_p95_pct: float

    # Usable energy reserve margin percentiles [%]
    reserve_margin_p05_pct: float
    reserve_margin_p50_pct: float
    reserve_margin_p95_pct: float

    # Feasibility and ranking
    is_statistically_feasible: bool
    rejection_reasons: List[str] = field(default_factory=list)
    predictive_score: float = 0.0
    selection_rank: int = 99

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class PredictiveRiskReport:
    """
    Comprehensive multi-route predictive risk comparison report.
    """
    timestamp_s: float
    scenario_count: int
    computation_time_ms: float
    status: str                         # FEASIBLE, CAUTION, CRITICAL, NO_FEASIBLE_ROUTE
    recommended_route_id: str
    recommended_destination: str
    explanation: str
    route_summaries: List[RouteRiskSummary]
    warnings: List[str] = field(default_factory=list)
    disclaimer: str = "SIMULATED ENSEMBLE PREDICTIONS — NOT LIVE MEASURED AVIONICS"

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["route_summaries"] = [r.to_dict() for r in self.route_summaries]
        return d


@dataclass
class PredictiveSimulationTelemetry:
    """
    Lightweight telemetry export object for Digital Twin state synchronization.
    Maintains complete separation from live measured flight telemetry.
    """
    prediction_status: str
    prediction_timestamp: float
    prediction_scenario_count: int
    predicted_mission_completion_rate: float
    predicted_rtb_feasibility_rate: float
    predicted_battery_depletion_rate: float
    predicted_engine_thermal_risk: float
    predicted_battery_thermal_risk: float
    predicted_fuel_reserve_violation_rate: float
    predicted_energy_reserve_p05: float
    predicted_energy_reserve_p95: float
    recommended_predictive_route: str
    prediction_computation_time_ms: float
    route_summaries: List[Dict[str, Any]] = field(default_factory=list)
    disclaimer: str = "SIMULATED ENSEMBLE PREDICTIONS — NOT LIVE MEASURED AVIONICS"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class PredictiveRiskAnalyzer:
    """
    Aggregates Monte Carlo scenario simulation records into risk frequencies,
    confidence intervals, and predictive trajectory selections.
    """

    def __init__(
        self,
        config: Optional[Union[FastForwardConfig, Path, str, Dict[str, Any]]] = None,
        ff_engine: Optional[FastForwardSimulationEngine] = None,
        scenario_gen: Optional[ScenarioGenerator] = None
    ):
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
            raise TypeError(f"Invalid config type for PredictiveRiskAnalyzer: {type(config)}")

        self.ff_engine = ff_engine if ff_engine is not None else FastForwardSimulationEngine(self.config)
        self.scenario_gen = scenario_gen if scenario_gen is not None else ScenarioGenerator(self.config)

    def evaluate_route_ensemble(
        self,
        candidate_route: CandidateRoute,
        initial_fuel_liters: float,
        initial_soc_pct: float,
        initial_soh_pct: float = 100.0,
        initial_battery_temp_c: float = 25.0,
        initial_engine_cht_c: float = 175.0,
        initial_engine_oil_t_c: float = 88.0,
        initial_motor_temp_c: float = 25.0,
        scenarios: Optional[List[ScenarioParameters]] = None,
        seed: Optional[int] = None
    ) -> RouteRiskSummary:
        """
        Executes fast-forward simulations for a single candidate route across the scenario ensemble.
        Aggregates outputs into statistical risk metrics and confidence bounds.
        """
        if scenarios is None:
            # Generate default ensemble: Nominal, Best, Worst, + Monte Carlo
            scenarios = self.scenario_gen.generate_full_ensemble(
                route_bearing_deg=0.0,
                include_deterministic_bounds=True,
                seed=seed
            )

        n_scenarios = len(scenarios)
        if n_scenarios == 0:
            raise ValueError("Scenario ensemble is empty.")

        records: List[ScenarioSimulationRecord] = []
        for scn in scenarios:
            rec = self.ff_engine.simulate_scenario(
                candidate_route=candidate_route,
                initial_fuel_liters=initial_fuel_liters,
                initial_soc_pct=initial_soc_pct,
                initial_soh_pct=initial_soh_pct,
                initial_battery_temp_c=initial_battery_temp_c,
                initial_engine_cht_c=initial_engine_cht_c,
                initial_engine_oil_t_c=initial_engine_oil_t_c,
                initial_motor_temp_c=initial_motor_temp_c,
                scenario=scn
            )
            records.append(rec)

        # 1. Frequency calculations
        completed_count = sum(1 for r in records if r.is_completed)
        completion_rate = (completed_count / n_scenarios) * 100.0
        wilson_ci = compute_wilson_score_interval(completed_count, n_scenarios)

        fuel_breach_count = sum(1 for r in records if r.has_fuel_reserve_breach)
        fuel_breach_rate = (fuel_breach_count / n_scenarios) * 100.0

        batt_breach_count = sum(1 for r in records if r.has_battery_reserve_breach)
        batt_breach_rate = (batt_breach_count / n_scenarios) * 100.0

        depletion_count = sum(1 for r in records if r.completion_status == "DEPLETED_BATTERY" or r.final_battery_soc_pct <= 0.5)
        depletion_rate = (depletion_count / n_scenarios) * 100.0

        thermal_count = sum(1 for r in records if r.has_thermal_violation)
        thermal_rate = (thermal_count / n_scenarios) * 100.0

        deficit_count = sum(1 for r in records if r.has_power_deficit_breach)
        deficit_rate = (deficit_count / n_scenarios) * 100.0

        # 2. Duration statistics (only for completed runs if possible, or total elapsed)
        durations_min = [(r.estimated_arrival_time_s if r.estimated_arrival_time_s is not None else r.simulated_elapsed_time_s) / 60.0 for r in records]
        mean_dur = sum(durations_min) / len(durations_min)
        var_dur = sum((d - mean_dur) ** 2 for d in durations_min) / max(1, len(durations_min) - 1)
        std_dur = math.sqrt(var_dur)

        dur_p05 = compute_percentile(durations_min, 5.0)
        dur_p50 = compute_percentile(durations_min, 50.0)
        dur_p95 = compute_percentile(durations_min, 95.0)

        # 3. Fuel statistics
        fuel_remaining = [r.final_fuel_remaining_liters for r in records]
        fuel_p05 = compute_percentile(fuel_remaining, 5.0)
        fuel_p50 = compute_percentile(fuel_remaining, 50.0)
        fuel_p95 = compute_percentile(fuel_remaining, 95.0)
        fuel_consumed = [r.total_fuel_consumed_liters for r in records]
        mean_fuel_consumed = sum(fuel_consumed) / len(fuel_consumed)

        # 4. Battery SoC statistics
        batt_socs = [r.final_battery_soc_pct for r in records]
        soc_p05 = compute_percentile(batt_socs, 5.0)
        soc_p50 = compute_percentile(batt_socs, 50.0)
        soc_p95 = compute_percentile(batt_socs, 95.0)

        # 5. Usable reserve margins
        margins = [r.arrival_reserve_margin_pct if r.arrival_reserve_margin_pct is not None else 0.0 for r in records]
        margin_p05 = compute_percentile(margins, 5.0)
        margin_p50 = compute_percentile(margins, 50.0)
        margin_p95 = compute_percentile(margins, 95.0)

        # 6. Statistical Feasibility Filter
        rejections = []
        is_stat_feasible = True

        if completion_rate < 85.0:
            is_stat_feasible = False
            rejections.append(f"Low completion probability ({completion_rate:.1f}% < 85.0% threshold)")

        if fuel_breach_rate > 20.0:
            is_stat_feasible = False
            rejections.append(f"Elevated fuel reserve violation risk ({fuel_breach_rate:.1f}% > 20.0% limit)")

        if depletion_rate > 5.0:
            is_stat_feasible = False
            rejections.append(f"Excessive battery depletion probability ({depletion_rate:.1f}% > 5.0% limit)")

        if thermal_rate > 15.0:
            is_stat_feasible = False
            rejections.append(f"Elevated thermal limit breach risk ({thermal_rate:.1f}% > 15.0% limit)")

        # Predictive Score (0 to 100, higher is better)
        # Weighted combination of completion rate, reserve margin, and low risk
        score = (
            completion_rate * 0.40 +
            max(0.0, min(100.0, margin_p50)) * 0.30 +
            (100.0 - fuel_breach_rate) * 0.15 +
            (100.0 - thermal_rate) * 0.15
        )
        if not is_stat_feasible:
            score = max(5.0, score * 0.35)

        return RouteRiskSummary(
            route_id=candidate_route.route_id,
            route_type=candidate_route.route_type,
            destination_name=candidate_route.destination_name,
            distance_km=round(candidate_route.total_distance_m / 1000.0, 1),
            scenario_count=n_scenarios,
            completion_rate_pct=round(completion_rate, 1),
            completion_ci_95_pct=wilson_ci,
            fuel_reserve_violation_rate_pct=round(fuel_breach_rate, 1),
            battery_reserve_violation_rate_pct=round(batt_breach_rate, 1),
            battery_depletion_rate_pct=round(depletion_rate, 1),
            thermal_risk_rate_pct=round(thermal_rate, 1),
            power_deficit_rate_pct=round(deficit_rate, 1),
            duration_mean_min=round(mean_dur, 1),
            duration_std_min=round(std_dur, 1),
            duration_p05_min=round(dur_p05, 1),
            duration_p50_min=round(dur_p50, 1),
            duration_p95_min=round(dur_p95, 1),
            fuel_remaining_p05_l=round(fuel_p05, 2),
            fuel_remaining_p50_l=round(fuel_p50, 2),
            fuel_remaining_p95_l=round(fuel_p95, 2),
            fuel_consumed_mean_l=round(mean_fuel_consumed, 2),
            battery_soc_p05_pct=round(soc_p05, 1),
            battery_soc_p50_pct=round(soc_p50, 1),
            battery_soc_p95_pct=round(soc_p95, 1),
            reserve_margin_p05_pct=round(margin_p05, 1),
            reserve_margin_p50_pct=round(margin_p50, 1),
            reserve_margin_p95_pct=round(margin_p95, 1),
            is_statistically_feasible=is_stat_feasible,
            rejection_reasons=rejections,
            predictive_score=round(score, 1),
            selection_rank=99
        )

    def compare_candidate_routes(
        self,
        candidate_routes: List[CandidateRoute],
        initial_fuel_liters: float,
        initial_soc_pct: float,
        initial_soh_pct: float = 100.0,
        initial_battery_temp_c: float = 25.0,
        initial_engine_cht_c: float = 175.0,
        initial_engine_oil_t_c: float = 88.0,
        initial_motor_temp_c: float = 25.0,
        scenarios: Optional[List[ScenarioParameters]] = None,
        seed: Optional[int] = None,
        timestamp_s: Optional[float] = None
    ) -> PredictiveRiskReport:
        """
        Evaluates and ranks all Step 8 candidate routes under identical scenario conditions.
        Enforces that lower average cost cannot override hard safety reserve constraints.
        """
        t0 = time.time()
        actual_seed = seed if seed is not None else self.config.random_seed

        if scenarios is None:
            # Use identical scenario realizations across all candidate routes
            scenarios = self.scenario_gen.generate_full_ensemble(
                route_bearing_deg=0.0,
                include_deterministic_bounds=True,
                seed=actual_seed
            )

        summaries: List[RouteRiskSummary] = []
        for cr in candidate_routes:
            summary = self.evaluate_route_ensemble(
                candidate_route=cr,
                initial_fuel_liters=initial_fuel_liters,
                initial_soc_pct=initial_soc_pct,
                initial_soh_pct=initial_soh_pct,
                initial_battery_temp_c=initial_battery_temp_c,
                initial_engine_cht_c=initial_engine_cht_c,
                initial_engine_oil_t_c=initial_engine_oil_t_c,
                initial_motor_temp_c=initial_motor_temp_c,
                scenarios=scenarios,
                seed=actual_seed
            )
            summaries.append(summary)

        # Rank routes: Statistically feasible routes first, sorted by predictive score descending
        feasible_routes = [s for s in summaries if s.is_statistically_feasible]
        infeasible_routes = [s for s in summaries if not s.is_statistically_feasible]

        feasible_routes.sort(key=lambda s: s.predictive_score, reverse=True)
        infeasible_routes.sort(key=lambda s: s.predictive_score, reverse=True)

        ranked = feasible_routes + infeasible_routes
        for idx, s in enumerate(ranked):
            s.selection_rank = idx + 1

        computation_time_ms = (time.time() - t0) * 1000.0

        # Formulate status & recommendation
        warnings: List[str] = []
        if len(feasible_routes) > 0:
            rec = feasible_routes[0]
            status = "FEASIBLE" if rec.completion_rate_pct >= 95.0 else "CAUTION"
            explanation = (
                f"Selected optimal predictive trajectory '{rec.route_id}' ({rec.destination_name}) "
                f"with {rec.completion_rate_pct:.1f}% ensemble completion rate (95% CI: [{rec.completion_ci_95_pct[0]}%, {rec.completion_ci_95_pct[1]}%]), "
                f"P50 arrival reserve {rec.fuel_remaining_p50_l:.1f}L / {rec.battery_soc_p50_pct:.1f}% SoC."
            )
        else:
            rec = ranked[0]
            status = "NO_FEASIBLE_ROUTE"
            warnings.append("No candidate path satisfies safety thresholds across the simulated uncertainty ensemble.")
            explanation = (
                f"WARNING: All candidate routes breached safety reserve limits under uncertainty. "
                f"Lowest-deficit path '{rec.route_id}' has {rec.completion_rate_pct:.1f}% completion rate "
                f"and {rec.fuel_reserve_violation_rate_pct:.1f}% fuel violation risk."
            )

        ts = timestamp_s if timestamp_s is not None else time.time()

        return PredictiveRiskReport(
            timestamp_s=round(ts, 2),
            scenario_count=len(scenarios),
            computation_time_ms=round(computation_time_ms, 1),
            status=status,
            recommended_route_id=rec.route_id,
            recommended_destination=rec.destination_name,
            explanation=explanation,
            route_summaries=ranked,
            warnings=warnings
        )

    def generate_telemetry_snapshot(
        self,
        report: PredictiveRiskReport
    ) -> PredictiveSimulationTelemetry:
        """
        Converts a PredictiveRiskReport into a lightweight telemetry object for Digital Twin streaming.
        """
        rec_summary = report.route_summaries[0] if report.route_summaries else None
        
        # Determine aggregate rates from recommended route
        comp_rate = rec_summary.completion_rate_pct if rec_summary else 0.0
        batt_dep = rec_summary.battery_depletion_rate_pct if rec_summary else 0.0
        fuel_vio = rec_summary.fuel_reserve_violation_rate_pct if rec_summary else 0.0
        therm_risk = rec_summary.thermal_risk_rate_pct if rec_summary else 0.0
        
        # RTB feasibility rate: find Direct RTB route if present
        rtb_summary = next((s for s in report.route_summaries if "RTB" in s.route_id), rec_summary)
        rtb_rate = rtb_summary.completion_rate_pct if rtb_summary else comp_rate

        res_p05 = rec_summary.reserve_margin_p05_pct if rec_summary else 0.0
        res_p95 = rec_summary.reserve_margin_p95_pct if rec_summary else 0.0

        return PredictiveSimulationTelemetry(
            prediction_status=report.status,
            prediction_timestamp=report.timestamp_s,
            prediction_scenario_count=report.scenario_count,
            predicted_mission_completion_rate=comp_rate,
            predicted_rtb_feasibility_rate=rtb_rate,
            predicted_battery_depletion_rate=batt_dep,
            predicted_engine_thermal_risk=round(therm_risk * 0.5, 1),
            predicted_battery_thermal_risk=round(therm_risk * 0.5, 1),
            predicted_fuel_reserve_violation_rate=fuel_vio,
            predicted_energy_reserve_p05=res_p05,
            predicted_energy_reserve_p95=res_p95,
            recommended_predictive_route=report.recommended_route_id,
            prediction_computation_time_ms=report.computation_time_ms,
            route_summaries=[s.to_dict() for s in report.route_summaries]
        )
