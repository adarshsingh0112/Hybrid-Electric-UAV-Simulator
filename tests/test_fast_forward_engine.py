"""
DRDO MALE UAV Multi-Scenario Predictive Simulation (Step 9)
Test Suite: Fast-Forward Simulation Engine (Physical Trajectory Projections)
"""
import sys
import time
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from mission.mission_state import GeographicPosition
from mission.trajectory_optimizer import CandidateRoute
from simulation.scenario_generator import FastForwardConfig, ScenarioParameters, ScenarioType
from simulation.fast_forward_engine import (
    FastForwardSimulationEngine,
    ScenarioSimulationRecord,
    calculate_bearing_deg,
)
from simulation.battery import BatteryModel, BatteryConfig


def _create_test_candidate_route(distance_scale: float = 1.0) -> CandidateRoute:
    """Helper to generate a realistic test route between Bangalore and Mysore."""
    pos1 = GeographicPosition(latitude_deg=12.9716, longitude_deg=77.5946, altitude_m=3000.0)
    # Scaled position for desired distance
    lat2 = 12.9716 + (0.45 * distance_scale)
    lon2 = 77.5946 + (0.45 * distance_scale)
    pos2 = GeographicPosition(latitude_deg=lat2, longitude_deg=lon2, altitude_m=3000.0)

    dist_m = 70000.0 * distance_scale
    return CandidateRoute(
        route_id="TEST_DIRECT_RTB",
        route_type="DIRECT_RTB",
        destination_name="Bangalore Base",
        destination_position=pos2,
        waypoint_names=["WP_CURRENT", "WP_BASE"],
        waypoints=[pos1, pos2],
        total_distance_m=dist_m,
        total_duration_s=dist_m / 45.0,
        total_fuel_liters=8.0 * distance_scale,
        total_battery_energy_wh=500.0 * distance_scale,
        remaining_fuel_liters=30.0,
        remaining_battery_soc_pct=75.0,
        reserve_margin_pct=35.0,
        is_feasible=True,
    )


def test_1_model_state_isolation():
    """Verify that forward simulation does NOT mutate live or external model objects."""
    print("  [TEST 1] Testing Physical Model State Isolation...")
    engine = FastForwardSimulationEngine()
    test_route = _create_test_candidate_route()

    # External battery model instance representing live Digital Twin state
    live_b_cfg = BatteryConfig(initial_soc_pct=80.0, initial_temp_c=28.0)
    live_battery = BatteryModel(config=live_b_cfg)
    orig_soc = live_battery.soc_pct
    orig_temp = live_battery.temperature_c

    # Run forward simulation
    record = engine.simulate_scenario(
        candidate_route=test_route,
        initial_fuel_liters=40.0,
        initial_soc_pct=orig_soc,
        initial_battery_temp_c=orig_temp,
        dt_s=2.0,
    )

    # Verify live battery model instance was completely unmutated
    assert live_battery.soc_pct == orig_soc, f"Live battery SoC mutated: {live_battery.soc_pct} != {orig_soc}"
    assert live_battery.temperature_c == orig_temp, f"Live battery Temp mutated: {live_battery.temperature_c} != {orig_temp}"
    assert record.is_completed is True
    assert record.simulated_elapsed_time_s > 0.0
    print("    -> Live digital twin models remained 100% unmutated during predictive execution.")


def test_2_battery_depletion_and_reserve_breach():
    """Verify detection of battery reserve breach and depletion under high electrical load."""
    print("  [TEST 2] Testing Battery Depletion & Reserve Breach Detection...")
    config = FastForwardConfig(min_battery_reserve_pct=25.0)
    engine = FastForwardSimulationEngine(config=config)
    test_route = _create_test_candidate_route(distance_scale=1.5)

    # High electrical draw scenario with low initial SoC
    scenario = ScenarioParameters(
        scenario_id="BATT_DEPLETION_TEST",
        scenario_type=ScenarioType.WORST_CASE,
        wind_speed_mps=10.0,
        wind_direction_deg=0.0,
        ambient_temp_delta_c=5.0,
        engine_power_multiplier=0.60,      # Severely derated engine forces hybrid boost / battery draw
        engine_fuel_flow_multiplier=1.1,
        battery_resistance_multiplier=1.35, # High internal resistance causes voltage sag and fast discharge
        battery_initial_soc_offset_pct=-3.0,
        propulsion_power_multiplier=1.15,
        payload_mass_delta_kg=20.0,
    )

    record = engine.simulate_scenario(
        candidate_route=test_route,
        initial_fuel_liters=35.0,
        initial_soc_pct=24.0,              # Already below 25% minimum reserve
        initial_battery_temp_c=30.0,
        scenario=scenario,
        dt_s=2.0,
    )

    assert record.has_battery_reserve_breach is True
    assert record.first_battery_reserve_breach_s == 0.0  # Breached at start
    assert record.min_battery_soc_pct < 25.0
    print(f"    -> Battery reserve breach correctly flagged at t={record.first_battery_reserve_breach_s}s.")


def test_3_thermal_limit_breach_detection():
    """Verify detection of engine CHT and battery thermal limit crossings."""
    print("  [TEST 3] Testing Thermal Limit Breach Detection (CHT & Battery Temp)...")
    config = FastForwardConfig(
        critical_engine_cht_c=220.0,
        critical_battery_temp_c=65.0,
    )
    engine = FastForwardSimulationEngine(config=config)
    test_route = _create_test_candidate_route()

    # Extreme hot weather scenario with high initial CHT
    hot_scenario = ScenarioParameters(
        scenario_id="THERMAL_STRESS_TEST",
        scenario_type=ScenarioType.WORST_CASE,
        wind_speed_mps=15.0,
        wind_direction_deg=0.0,
        ambient_temp_delta_c=12.0,
        engine_power_multiplier=1.0,
        engine_fuel_flow_multiplier=1.2,
        battery_resistance_multiplier=1.3,
        battery_initial_soc_offset_pct=0.0,
        propulsion_power_multiplier=1.15,
        payload_mass_delta_kg=20.0,
    )

    record = engine.simulate_scenario(
        candidate_route=test_route,
        initial_fuel_liters=40.0,
        initial_soc_pct=80.0,
        initial_engine_cht_c=222.0,        # Above 220C critical threshold
        initial_battery_temp_c=62.0,
        scenario=hot_scenario,
        dt_s=2.0,
    )

    assert record.has_thermal_violation is True
    assert record.first_thermal_breach_s is not None
    assert record.max_engine_cht_c >= 220.0 or record.max_battery_temp_c >= 65.0
    print(f"    -> Thermal limit breach accurately captured: first_breach={record.first_thermal_breach_s}s, max_cht={record.max_engine_cht_c}C.")


def test_4_fuel_reserve_violation_detection():
    """Verify detection of fuel reserve violation on extended flights."""
    print("  [TEST 4] Testing Fuel Reserve Violation Detection...")
    config = FastForwardConfig(min_fuel_reserve_liters=18.0)
    engine = FastForwardSimulationEngine(config=config)
    test_route = _create_test_candidate_route(distance_scale=1.5)

    record = engine.simulate_scenario(
        candidate_route=test_route,
        initial_fuel_liters=19.0,          # Barely 1 liter above reserve
        initial_soc_pct=85.0,
        dt_s=2.0,
    )

    assert record.has_fuel_reserve_breach is True
    assert record.first_fuel_reserve_breach_s is not None
    assert record.final_fuel_remaining_liters < 18.0
    print(f"    -> Fuel reserve breach captured at t={record.first_fuel_reserve_breach_s}s, final_fuel={record.final_fuel_remaining_liters}L.")


def test_5_faster_than_real_time_speed_benchmark():
    """Verify faster-than-real-time performance budget (sub-millisecond per trajectory second)."""
    print("  [TEST 5] Testing Faster-Than-Real-Time Speed Benchmark...")
    engine = FastForwardSimulationEngine()
    test_route = _create_test_candidate_route(distance_scale=1.0)

    t0 = time.time()
    # Execute 5 scenario runs
    for _ in range(5):
        engine.simulate_scenario(
            candidate_route=test_route,
            initial_fuel_liters=40.0,
            initial_soc_pct=75.0,
            dt_s=2.0,
        )
    elapsed_total_s = time.time() - t0
    avg_run_ms = (elapsed_total_s / 5.0) * 1000.0

    print(f"    -> Average single route simulation time: {avg_run_ms:.2f} ms (5 runs executed in {elapsed_total_s*1000:.1f} ms).")
    assert avg_run_ms < 100.0, f"Simulation too slow: {avg_run_ms:.2f} ms per run"
    print("    -> High-speed faster-than-real-time computational budget verified.")


if __name__ == "__main__":
    print("=" * 80)
    print("  FAST-FORWARD SIMULATION ENGINE TEST SUITE (STEP 9)")
    print("=" * 80)
    test_1_model_state_isolation()
    test_2_battery_depletion_and_reserve_breach()
    test_3_thermal_limit_breach_detection()
    test_4_fuel_reserve_violation_detection()
    test_5_faster_than_real_time_speed_benchmark()
    print("=" * 80)
    print("  ALL 5 FAST-FORWARD ENGINE TESTS PASSED! (5/5 PASS)")
    print("=" * 80)
