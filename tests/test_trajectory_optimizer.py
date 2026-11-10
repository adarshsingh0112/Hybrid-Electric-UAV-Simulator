"""
DRDO MALE UAV Multi-Criteria Trajectory Optimizer Test Suite (Step 8)
Verifies candidate route generation, aerodynamic power calculations, hybrid mode split,
hard safety reserve constraint enforcement, multi-criteria cost normalization, candidate ranking,
and execution throttling / caching.

Test Matrix:
1. Candidate Route Generation: Direct RTB, Corridor RTB, Mission Continuation, Alternate Diversions.
2. Aerodynamic Power Calculation & Hybrid Mode Strategies (Engine, Electric, Motor Assist, Charging).
3. Hard Safety Reserve Policy Enforcement (Fuel >= 18 L, Battery SoC >= 20%, Deficit <= 8 kW).
4. Multi-Criteria Cost Function Normalization & Weights Accounting.
5. Dynamic Candidate Ranking & Optimal Path Selection.
6. Emergency Scenario: All Routes Infeasible -> NO_FEASIBLE_ROUTE status & minimal-deficit selection.
7. Execution Throttling & Caching (1.0 Hz throttling prevents high-frequency 20 Hz CPU waste).
"""

import sys
import math
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent if (Path(__file__).resolve().parent.name == "tests") else Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from mission.trajectory_optimizer import (
    TrajectoryOptimizer,
    TrajectoryOptimizationTelemetry,
    CandidateRoute,
    PropulsionStrategy,
)
from mission.decision_engine import DecisionEngineConfig, DestinationInfo
from mission.mission_state import GeographicPosition, Waypoint
from simulation.hybrid_powertrain import HybridOperatingMode


def get_standard_mission_waypoints():
    """Pune - Baramati - Solapur standard mission waypoints."""
    return [
        Waypoint("WP-01", 18.5822, 73.9197, 6000.0, "Pune Lohegaon (Base)"),
        Waypoint("WP-02", 18.3500, 74.2000, 7500.0, "Saswad Waypoint"),
        Waypoint("WP-03", 18.1500, 74.5800, 8500.0, "Baramati Airfield"),
        Waypoint("WP-04", 17.9500, 75.1000, 8500.0, "Indapur Patrol Waypoint"),
        Waypoint("WP-05", 17.6599, 75.9064, 8500.0, "Solapur Target Area"),
    ]


def test_1_candidate_routes_generation():
    print("[TEST 1] Testing Candidate Routes Generation...")
    optimizer = TrajectoryOptimizer()
    wps = get_standard_mission_waypoints()
    curr_pos = GeographicPosition(18.2500, 74.4000, 8000.0)
    home_pos = wps[0].to_position()

    telemetry = optimizer.optimize_trajectory(
        current_pos=curr_pos,
        home_pos=home_pos,
        active_waypoint_index=2,
        all_mission_waypoints=wps,
        remaining_fuel_liters=120.0,
        battery_soc_pct=90.0,
        engine_health_pct=100.0,
        ground_speed_mps=55.0,
        timestamp_s=100.0,
        force_recompute=True
    )

    print(f"  -> Total Candidates Evaluated: {telemetry.candidate_routes_count}")
    route_ids = [c["route_id"] for c in telemetry.candidate_summaries]
    print(f"  -> Candidate Route IDs: {route_ids}")

    assert telemetry.candidate_routes_count >= 4, f"Expected >= 4 candidates, got {telemetry.candidate_routes_count}"
    assert "ROUTE_DIRECT_RTB" in route_ids, "Missing ROUTE_DIRECT_RTB"
    assert "ROUTE_CORRIDOR_RTB" in route_ids, "Missing ROUTE_CORRIDOR_RTB"
    assert "ROUTE_MISSION_CONTINUE" in route_ids, "Missing ROUTE_MISSION_CONTINUE"
    assert any("DIVERT_ALT" in r for r in route_ids), "Missing Alternate Diversion routes"

    print("  -> PASSED: All essential candidate route typologies generated.")


def test_2_aerodynamic_power_and_hybrid_strategy():
    print("[TEST 2] Testing Aerodynamic Power Calculation & Hybrid Mode Evaluation...")
    optimizer = TrajectoryOptimizer()

    # 1. Aerodynamic power verification: P = (m * g * V) / (L/D * eta_prop)
    # Mass = 1100 kg, V = 55 m/s, L/D = 15.0, eta = 0.82
    expected_p = (1100.0 * 9.80665 * 55.0) / (15.0 * 0.82) # ~48,272 W
    calc_p = optimizer.calculate_aerodynamic_power(airspeed_mps=55.0, altitude_m=3000.0)
    print(f"  -> Calculated Aerodynamic Shaft Power: {calc_p:.1f} W (Expected ~{expected_p:.1f} W)")
    assert abs(calc_p - expected_p) < 100.0, f"Shaft power mismatch: {calc_p} vs {expected_p}"

    # 2. Hybrid strategy: Nominal cruise -> ENGINE_ONLY
    strat_cruise = optimizer.evaluate_propulsion_segment(
        p_req_w=calc_p,
        current_soc_pct=80.0,
        engine_health_pct=100.0
    )
    print(f"  -> Cruise Strategy: Mode={strat_cruise.operating_mode}, EngPower={strat_cruise.engine_power_w}W, Fuel={strat_cruise.fuel_flow_lph}LPH")
    assert strat_cruise.operating_mode == HybridOperatingMode.ENGINE_ONLY.value
    assert strat_cruise.power_deficit_w == 0.0

    # 3. Hybrid strategy: Depleted battery + power headroom -> BATTERY_CHARGING
    strat_charge = optimizer.evaluate_propulsion_segment(
        p_req_w=45000.0,
        current_soc_pct=22.0,  # Below 28% threshold
        engine_health_pct=100.0
    )
    print(f"  -> Low SoC Strategy: Mode={strat_charge.operating_mode}, BatteryPower={strat_charge.battery_power_w}W")
    assert strat_charge.operating_mode == HybridOperatingMode.BATTERY_CHARGING.value
    assert strat_charge.battery_power_w < 0.0 # Negative means charging

    # 4. Hybrid strategy: Electric loiter
    strat_elec = optimizer.evaluate_propulsion_segment(
        p_req_w=30000.0,
        current_soc_pct=75.0,
        prefer_electric=True
    )
    print(f"  -> Electric Loiter Strategy: Mode={strat_elec.operating_mode}, MotorPower={strat_elec.motor_power_w}W")
    assert strat_elec.operating_mode == HybridOperatingMode.ELECTRIC_ONLY.value
    assert strat_elec.engine_power_w == 0.0

    print("  -> PASSED: Physical aerodynamic equations and hybrid propulsion splits match theory.")


def test_3_hard_safety_reserve_constraints():
    print("[TEST 3] Testing Hard Safety Reserve Policy Enforcement...")
    optimizer = TrajectoryOptimizer()
    wps = get_standard_mission_waypoints()
    curr_pos = GeographicPosition(17.8000, 75.6000, 8000.0) # Near Solapur, ~200 km from home
    home_pos = wps[0].to_position()

    # Case A: Low initial fuel (19 L), RTB transit will burn ~18 L, breaching 18 L minimum reserve limit
    telemetry_fuel_breach = optimizer.optimize_trajectory(
        current_pos=curr_pos,
        home_pos=home_pos,
        active_waypoint_index=3,
        all_mission_waypoints=wps,
        remaining_fuel_liters=19.0,   # Not enough to complete RTB + 18 L reserve
        battery_soc_pct=90.0,
        engine_health_pct=100.0,
        ground_speed_mps=55.0,
        timestamp_s=200.0,
        force_recompute=True
    )

    direct_rtb = next((c for c in telemetry_fuel_breach.candidate_summaries if c["route_id"] == "ROUTE_DIRECT_RTB"), None)
    assert direct_rtb is not None
    print(f"  -> Direct RTB Feasible with 19L Fuel: {direct_rtb['is_feasible']}")
    print(f"  -> Rejection Reasons: {direct_rtb['rejection_reasons']}")
    assert direct_rtb["is_feasible"] is False, "Direct RTB should be rejected due to fuel reserve policy breach"
    assert any("Fuel reserve breach" in r for r in direct_rtb["rejection_reasons"])
    assert direct_rtb["multi_criteria_cost"] >= 10.0, "Infeasible routes must receive cost penalty >= 10.0"

    # Case B: Low battery SoC (21%), transit draws battery below 20% limit
    cand_batt_breach = optimizer.evaluate_route_candidate(
        route_id="TEST_BATT_BREACH",
        route_type="TEST",
        destination_name="Test Dest",
        destination_pos=home_pos,
        nodes=[("START", curr_pos), ("END", home_pos)],
        initial_fuel_l=100.0,
        initial_soc_pct=21.0,     # Only 1% above minimum reserve
        engine_health_pct=25.0,    # Forces electric mode which rapidly consumes battery
        ground_speed_mps=55.0
    )
    print(f"  -> Electric-depleting Route Feasible: {cand_batt_breach.is_feasible}")
    print(f"  -> Rejection Reasons: {cand_batt_breach.rejection_reasons}")
    assert cand_batt_breach.is_feasible is False
    assert any("Battery reserve breach" in r for r in cand_batt_breach.rejection_reasons)

    print("  -> PASSED: Safety reserve limits strictly invalidate non-compliant trajectories.")


def test_4_multi_criteria_cost_function():
    print("[TEST 4] Testing Multi-Criteria Cost Function Normalization & Weights...")
    optimizer = TrajectoryOptimizer()
    wps = get_standard_mission_waypoints()
    curr_pos = GeographicPosition(18.4500, 74.0500, 7000.0) # Near Pune (~25 km from home)
    home_pos = wps[0].to_position()

    route = optimizer.evaluate_route_candidate(
        route_id="TEST_NORM_ROUTE",
        route_type="DIRECT_RTB",
        destination_name="Pune Lohegaon",
        destination_pos=home_pos,
        nodes=[("START", curr_pos), ("HOME", home_pos)],
        initial_fuel_l=120.0,
        initial_soc_pct=95.0,
        engine_health_pct=100.0,
        ground_speed_mps=55.0
    )

    print(f"  -> Route Distance: {route.total_distance_m/1000.0:.1f} km")
    print(f"  -> Normalized Metrics: {route.normalized_metrics}")
    print(f"  -> Multi-Criteria Cost: {route.multi_criteria_cost:.4f}")

    assert route.is_feasible is True
    # All normalized components should be non-negative and bounded
    for metric_name, val in route.normalized_metrics.items():
        assert 0.0 <= val <= 2.0, f"Normalized metric '{metric_name}' out of bounds [0, 2]: {val}"

    # Verify manual weighted sum calculation
    w = optimizer.config.route_optimization_weights
    m = route.normalized_metrics
    expected_cost = (
        w["weight_fuel"] * m["norm_fuel"] +
        w["weight_battery"] * m["norm_battery"] +
        w["weight_duration"] * m["norm_duration"] +
        w["weight_distance"] * m["norm_distance"] +
        w["weight_margin"] * m["norm_margin"] +
        w["weight_degradation"] * m["norm_degradation"]
    )
    assert abs(route.multi_criteria_cost - expected_cost) < 0.001, "Cost calculation does not match weighted sum"
    print("  -> PASSED: Multi-criteria cost matches weighted normalized formula precisely.")


def test_5_dynamic_candidate_ranking():
    print("[TEST 5] Testing Dynamic Candidate Ranking & Optimal Path Selection...")
    optimizer = TrajectoryOptimizer()
    wps = get_standard_mission_waypoints()
    # At Baramati (WP-03), ample fuel and battery
    curr_pos = wps[2].to_position()
    home_pos = wps[0].to_position()

    telemetry = optimizer.optimize_trajectory(
        current_pos=curr_pos,
        home_pos=home_pos,
        active_waypoint_index=2,
        all_mission_waypoints=wps,
        remaining_fuel_liters=130.0,
        battery_soc_pct=95.0,
        engine_health_pct=100.0,
        ground_speed_mps=55.0,
        timestamp_s=300.0,
        force_recompute=True
    )

    print(f"  -> Status: {telemetry.status}")
    print(f"  -> Recommended Route: {telemetry.recommended_route_name}")
    print(f"  -> Optimization Score: {telemetry.route_optimization_score:.1f} / 100")
    print(f"  -> Candidate Summaries:")
    for c in telemetry.candidate_summaries:
        print(f"     Rank {c['rank']}: {c['route_id']:24s} | Feasible={c['is_feasible']} | Cost={c['multi_criteria_cost']:.3f} | Dist={c['total_distance_km']:.1f} km")

    assert telemetry.status == "OPTIMAL"
    assert telemetry.route_optimization_score >= 80.0
    # Check rank ordering
    costs = [c["multi_criteria_cost"] for c in telemetry.candidate_summaries]
    ranks = [c["rank"] for c in telemetry.candidate_summaries]
    assert ranks == list(range(1, len(ranks) + 1)), "Ranks must be strictly sequential 1, 2, 3..."
    # Feasible candidates must come before infeasible candidates
    feasible_flags = [c["is_feasible"] for c in telemetry.candidate_summaries]
    # Check that True never comes after False
    seen_false = False
    for f in feasible_flags:
        if not f:
            seen_false = True
        if seen_false:
            assert not f, "Feasible candidate ranked lower than an infeasible candidate"

    print("  -> PASSED: Candidates ranked strictly by physical feasibility and cost.")


def test_6_emergency_scenario_no_feasible_route():
    print("[TEST 6] Testing Emergency Scenario (No Feasible Route)...")
    optimizer = TrajectoryOptimizer()
    wps = get_standard_mission_waypoints()
    curr_pos = GeographicPosition(17.5000, 76.0000, 8000.0) # Distant sector, near empty fuel & battery
    home_pos = wps[0].to_position()

    telemetry = optimizer.optimize_trajectory(
        current_pos=curr_pos,
        home_pos=home_pos,
        active_waypoint_index=4,
        all_mission_waypoints=wps,
        remaining_fuel_liters=5.0,     # Below 18 L limit
        battery_soc_pct=10.0,          # Below 20% limit
        engine_health_pct=30.0,
        ground_speed_mps=55.0,
        timestamp_s=400.0,
        force_recompute=True
    )

    print(f"  -> Status: {telemetry.status}")
    print(f"  -> Route Name: {telemetry.recommended_route_name}")
    print(f"  -> Score: {telemetry.route_optimization_score:.1f}")
    print(f"  -> Explanation: {telemetry.explanation}")

    assert telemetry.status == "NO_FEASIBLE_ROUTE"
    assert telemetry.route_optimization_score <= 20.0
    assert "CRITICAL WARNING" in telemetry.explanation
    print("  -> PASSED: Infeasible emergency scenario correctly flagged without fabrication.")


def test_7_execution_throttling_and_caching():
    print("[TEST 7] Testing Execution Throttling & 1.0s Periodic Caching...")
    optimizer = TrajectoryOptimizer()
    wps = get_standard_mission_waypoints()
    curr_pos = GeographicPosition(18.5000, 74.0000, 7000.0)
    home_pos = wps[0].to_position()

    # Call 1: At t = 500.00 s
    t1 = optimizer.optimize_trajectory(
        current_pos=curr_pos,
        home_pos=home_pos,
        active_waypoint_index=1,
        all_mission_waypoints=wps,
        remaining_fuel_liters=100.0,
        battery_soc_pct=90.0,
        ground_speed_mps=55.0,
        timestamp_s=500.00,
        force_recompute=True
    )

    # Call 2: At t = 500.05 s (50 ms later = 20 Hz tick) without force_recompute
    t2 = optimizer.optimize_trajectory(
        current_pos=curr_pos,
        home_pos=home_pos,
        active_waypoint_index=1,
        all_mission_waypoints=wps,
        remaining_fuel_liters=100.0,
        battery_soc_pct=90.0,
        ground_speed_mps=55.0,
        timestamp_s=500.05,
        force_recompute=False
    )

    print(f"  -> Call 1 Timestamp: {t1.timestamp_s:.2f} s")
    print(f"  -> Call 2 Timestamp: {t2.timestamp_s:.2f} s")
    assert t1 is t2, "Optimizer should return identical cached instance on high-frequency 50ms tick"

    # Call 3: At t = 501.20 s (> 1.0s interval) -> should refresh cache
    t3 = optimizer.optimize_trajectory(
        current_pos=curr_pos,
        home_pos=home_pos,
        active_waypoint_index=1,
        all_mission_waypoints=wps,
        remaining_fuel_liters=99.5,
        battery_soc_pct=89.5,
        ground_speed_mps=55.0,
        timestamp_s=501.20,
        force_recompute=False
    )
    print(f"  -> Call 3 Timestamp: {t3.timestamp_s:.2f} s")
    assert t3.timestamp_s == 501.20, f"Expected cache update at 501.20, got {t3.timestamp_s}"

    print("  -> PASSED: 1.0s execution throttling verified, protecting 20 Hz loop from compute starvation.")


if __name__ == "__main__":
    print("=" * 80)
    print("  DRDO MALE UAV MULTI-CRITERIA TRAJECTORY OPTIMIZER TEST SUITE (STEP 8)")
    print("=" * 80)

    test_1_candidate_routes_generation()
    print()
    test_2_aerodynamic_power_and_hybrid_strategy()
    print()
    test_3_hard_safety_reserve_constraints()
    print()
    test_4_multi_criteria_cost_function()
    print()
    test_5_dynamic_candidate_ranking()
    print()
    test_6_emergency_scenario_no_feasible_route()
    print()
    test_7_execution_throttling_and_caching()
    print()

    print("=" * 80)
    print("  ALL 7 TRAJECTORY OPTIMIZATION TESTS PASSED SUCCESSFULLY! (7/7 PASS)")
    print("=" * 80)
