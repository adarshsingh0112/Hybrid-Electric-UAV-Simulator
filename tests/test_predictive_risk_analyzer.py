"""
DRDO MALE UAV Multi-Scenario Predictive Simulation (Step 9)
Test Suite: Predictive Risk Analyzer (Statistical Aggregation & Risk Reporting)
"""
import sys
import math
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from mission.mission_state import GeographicPosition
from mission.trajectory_optimizer import CandidateRoute
from simulation.scenario_generator import FastForwardConfig
from mission.predictive_risk_analyzer import (
    compute_percentile,
    compute_wilson_score_interval,
    RouteRiskSummary,
    PredictiveRiskReport,
    PredictiveSimulationTelemetry,
    PredictiveRiskAnalyzer,
)


def _create_mock_route(route_id: str, dist_km: float, is_feasible: bool = True) -> CandidateRoute:
    """Helper to generate mock candidate route for risk evaluation."""
    pos1 = GeographicPosition(latitude_deg=12.9716, longitude_deg=77.5946, altitude_m=3000.0)
    pos2 = GeographicPosition(latitude_deg=12.9716 + (dist_km / 111.0), longitude_deg=77.5946, altitude_m=3000.0)
    dist_m = dist_km * 1000.0

    return CandidateRoute(
        route_id=route_id,
        route_type="DIRECT_RTB" if "RTB" in route_id else "MISSION_CONTINUE",
        destination_name="Bangalore Base" if "RTB" in route_id else "Waypoint 4",
        destination_position=pos2,
        waypoint_names=["WP_START", "WP_END"],
        waypoints=[pos1, pos2],
        total_distance_m=dist_m,
        total_duration_s=dist_m / 45.0,
        total_fuel_liters=dist_km * 0.12,
        total_battery_energy_wh=dist_km * 10.0,
        remaining_fuel_liters=35.0,
        remaining_battery_soc_pct=70.0,
        reserve_margin_pct=30.0,
        is_feasible=is_feasible,
    )


def test_1_statistical_percentiles_and_wilson_ci():
    """Verify linear interpolation percentiles and Wilson score confidence intervals."""
    print("  [TEST 1] Testing Statistical Percentiles & Wilson Score Intervals...")
    values = [10.0, 20.0, 30.0, 40.0, 50.0]
    p50 = compute_percentile(values, 50.0)
    p00 = compute_percentile(values, 0.0)
    p100 = compute_percentile(values, 100.0)
    p05 = compute_percentile(values, 5.0)
    p95 = compute_percentile(values, 95.0)

    assert abs(p50 - 30.0) < 1e-5
    assert abs(p00 - 10.0) < 1e-5
    assert abs(p100 - 50.0) < 1e-5
    assert 10.0 <= p05 <= 20.0
    assert 40.0 <= p95 <= 50.0

    # Wilson interval testing
    # Zero successes
    low0, high0 = compute_wilson_score_interval(0, 20)
    assert low0 == 0.0
    assert high0 > 0.0 and high0 < 25.0

    # 100% successes
    low100, high100 = compute_wilson_score_interval(20, 20)
    assert low100 > 75.0
    assert high100 == 100.0

    # 50% successes
    low50, high50 = compute_wilson_score_interval(10, 20)
    assert 25.0 < low50 < 50.0
    assert 50.0 < high50 < 75.0
    print("    -> Percentiles and Wilson score intervals mathematically verified.")


def test_2_route_ensemble_evaluation():
    """Verify ensemble evaluation yields complete statistical summary."""
    print("  [TEST 2] Testing Single Route Ensemble Evaluation...")
    cfg = FastForwardConfig(num_scenarios=10, simulation_timestep_s=2.0)
    analyzer = PredictiveRiskAnalyzer(config=cfg)
    route = _create_mock_route("TEST_ROUTE_A", dist_km=40.0)

    summary = analyzer.evaluate_route_ensemble(
        candidate_route=route,
        initial_fuel_liters=40.0,
        initial_soc_pct=80.0,
        seed=42,
    )

    assert summary.route_id == "TEST_ROUTE_A"
    assert summary.scenario_count == 13  # 1 nominal + 2 bounds + 10 MC
    assert 0.0 <= summary.completion_rate_pct <= 100.0
    assert summary.fuel_remaining_p05_l <= summary.fuel_remaining_p50_l <= summary.fuel_remaining_p95_l
    assert summary.battery_soc_p05_pct <= summary.battery_soc_p50_pct <= summary.battery_soc_p95_pct
    assert summary.reserve_margin_p05_pct <= summary.reserve_margin_p50_pct <= summary.reserve_margin_p95_pct
    assert summary.duration_p05_min <= summary.duration_p50_min <= summary.duration_p95_min
    assert summary.is_statistically_feasible is True
    print(f"    -> Route A evaluated across {summary.scenario_count} scenarios. Completion: {summary.completion_rate_pct}%.")


def test_3_multi_route_comparison_identical_scenarios():
    """Verify multi-route comparison ranks routes deterministically under identical uncertainty."""
    print("  [TEST 3] Testing Multi-Route Comparison Under Identical Uncertainty...")
    cfg = FastForwardConfig(num_scenarios=8, simulation_timestep_s=2.0)
    analyzer = PredictiveRiskAnalyzer(config=cfg)

    short_route = _create_mock_route("RTB_DIRECT", dist_km=25.0)
    long_route = _create_mock_route("MISSION_LONG", dist_km=180.0)

    report = analyzer.compare_candidate_routes(
        candidate_routes=[long_route, short_route],
        initial_fuel_liters=30.0,
        initial_soc_pct=70.0,
        seed=101,
        timestamp_s=150.0,
    )

    assert len(report.route_summaries) == 2
    assert report.timestamp_s == 150.0
    # Short route should have higher completion and rank 1
    assert report.route_summaries[0].route_id == "RTB_DIRECT"
    assert report.route_summaries[0].selection_rank == 1
    assert report.recommended_route_id == "RTB_DIRECT"
    print(f"    -> Recommended route: {report.recommended_route_id} with rank 1.")


def test_4_hard_safety_reserve_precedence():
    """Verify that routes breaching safety thresholds are rejected despite any nominal appeal."""
    print("  [TEST 4] Testing Hard Safety Reserve Precedence Over Nominal Cost...")
    cfg = FastForwardConfig(
        num_scenarios=8,
        min_fuel_reserve_liters=18.0,
        simulation_timestep_s=2.0,
    )
    analyzer = PredictiveRiskAnalyzer(config=cfg)

    # Route that consumes too much fuel with marginal initial fuel
    risky_route = _create_mock_route("RISKY_ROUTE", dist_km=120.0)

    summary = analyzer.evaluate_route_ensemble(
        candidate_route=risky_route,
        initial_fuel_liters=19.0,          # Only 1 liter above 18L reserve
        initial_soc_pct=80.0,
        seed=42,
    )

    # Must be marked statistically infeasible due to fuel violation
    assert summary.fuel_reserve_violation_rate_pct > 0.0
    assert summary.is_statistically_feasible is False
    assert len(summary.rejection_reasons) > 0
    print(f"    -> Safety constraint enforced: Risky route rejected with reasons: {summary.rejection_reasons}")


def test_5_telemetry_snapshot_and_disclaimer():
    """Verify telemetry snapshot generation and mandatory aerospace prediction disclaimer."""
    print("  [TEST 5] Testing Telemetry Snapshot Generation & Aerospace Disclaimers...")
    cfg = FastForwardConfig(num_scenarios=6, simulation_timestep_s=2.0)
    analyzer = PredictiveRiskAnalyzer(config=cfg)

    route = _create_mock_route("RTB_DIRECT", dist_km=30.0)
    report = analyzer.compare_candidate_routes(
        candidate_routes=[route],
        initial_fuel_liters=35.0,
        initial_soc_pct=75.0,
        seed=42,
        timestamp_s=200.0,
    )

    telem = analyzer.generate_telemetry_snapshot(report)
    assert telem.prediction_timestamp == 200.0
    assert telem.prediction_scenario_count == 9  # 1 nominal + 2 bounds + 6 MC
    assert telem.disclaimer == "SIMULATED ENSEMBLE PREDICTIONS — NOT LIVE MEASURED AVIONICS"
    assert "RTB_DIRECT" in telem.recommended_predictive_route
    assert len(telem.route_summaries) == 1

    d = telem.to_dict()
    assert "prediction_status" in d
    assert "predicted_energy_reserve_p05" in d
    assert "predicted_energy_reserve_p95" in d
    print("    -> Telemetry export and aerospace disclaimers strictly validated.")


if __name__ == "__main__":
    print("=" * 80)
    print("  PREDICTIVE RISK ANALYZER TEST SUITE (STEP 9)")
    print("=" * 80)
    test_1_statistical_percentiles_and_wilson_ci()
    test_2_route_ensemble_evaluation()
    test_3_multi_route_comparison_identical_scenarios()
    test_4_hard_safety_reserve_precedence()
    test_5_telemetry_snapshot_and_disclaimer()
    print("=" * 80)
    print("  ALL 5 PREDICTIVE RISK ANALYZER TESTS PASSED! (5/5 PASS)")
    print("=" * 80)
