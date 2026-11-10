"""
DRDO MALE UAV Mission Reliability Scoring & Dynamic Contingency Analysis Test Suite (Step 7)
Verification of explainable 0–100 reliability scoring, RTB feasibility prediction,
safety reserve policy enforcement, and deterministic contingency actions.

Test Matrix:
1. Normal Healthy Mission: High reliability index, status NOMINAL, CONTINUE_MISSION.
2. Low Battery Reserve: SoC < 20% limit triggers ABORT_AND_RTB.
3. Low Fuel Reserve: Fuel < 18 L limit triggers ABORT_AND_RTB.
4. Degraded Engine Health: Health < 65% triggers ABORT_AND_RTB.
5. Insufficient Return Energy: Infeasible RTB triggers DIVERT_TO_ALTERNATE or EMERGENCY_DESCENT.
6. Invalid Telemetry Injection: NaN values flag INVALID, clamp score <= 10, never CONTINUE_MISSION.
7. High Propulsion Power Demand: Severe power deficit (>8 kW) penalizes score and triggers ABORT_AND_RTB.
8. Zero / Low Ground Speed: Safeguards against ZeroDivisionError and uses cruise fallback.
9. Emergency Contingency: Dual propulsion loss or thermal critical triggers EMERGENCY_DESCENT.
10. Missing / Stale Subsystem Data: Lag > 200ms flags DEGRADED, applies penalty, triggers ABORT_AND_RTB.
"""

import sys
import math
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from simulation.flight_profile import FlightProfile, FlightState
from simulation.hybrid_powertrain import HybridPowertrain, HybridOperatingMode
from digital_twin.uav_digital_twin import (
    UAVDigitalTwin,
    UAVDigitalTwinValidator,
    DigitalTwinQualityStatus,
)
from mission.mission_state import GeographicPosition
from mission.reliability_engine import (
    MissionReliabilityEngine,
    ReliabilityConfig,
    ContingencyAction,
    ReliabilityStatusLevel,
)


def get_standard_flight_state(altitude_ft: float = 20000.0, airspeed_mps: float = 55.0) -> FlightState:
    alt_m = FlightProfile.feet_to_meters(altitude_ft)
    t_c, p_bar, rho = FlightProfile.get_isa_atmosphere(alt_m)
    return FlightState(
        time_s=100.0,
        altitude_ft=altitude_ft,
        altitude_m=alt_m,
        throttle_pct=70.0,
        airspeed_mps=airspeed_mps,
        ambient_temp_c=t_c,
        ambient_pressure_bar=p_bar,
        air_density_kgpm3=rho,
        phase="CRUISE"
    )


def test_1_normal_healthy_mission():
    print("[TEST 1] Testing Normal Healthy Mission Sortie...")
    powertrain = HybridPowertrain()
    digital_twin = UAVDigitalTwin()
    flight = get_standard_flight_state(15000.0)

    # Execute 30 steps of healthy flight
    snapshot = None
    for _ in range(30):
        ht = powertrain.update(
            propulsion_power_request_w=55000.0,
            flight_state=flight,
            ambient_temp_c=15.0,
            dt_s=0.05,
            mode_override=HybridOperatingMode.ENGINE_ONLY
        )
        snapshot = digital_twin.update_from_hybrid_telemetry(
            hybrid_telemetry=ht,
            flight_state=flight,
            engine_health_pct=98.5
        )

    rel = snapshot.reliability
    assert rel is not None, "Reliability state must not be None"
    print(f"  -> Mission Reliability Index: {rel.mission_reliability_index:.1f}%")
    print(f"  -> Reliability Status Level:  {rel.reliability_status}")
    print(f"  -> Contingency Decision:      {rel.contingency_recommendation}")
    print(f"  -> RTB Feasibility:           {rel.rtb_feasible}")
    print(f"  -> Energy Margin Buffer:      +{rel.available_energy_margin_pct:.1f}%")
    print(f"  -> Composite Sub-Index:       {rel.score_breakdown.composite_score:.1f}")

    assert rel.mission_reliability_index >= 85.0, f"Expected MRI >= 85, got {rel.mission_reliability_index}"
    assert rel.reliability_status == ReliabilityStatusLevel.NOMINAL.value
    assert rel.contingency_recommendation == ContingencyAction.CONTINUE_MISSION.value
    assert rel.rtb_feasible is True
    assert rel.available_energy_margin_pct > 15.0
    print("  -> PASSED: Nominal flight produces high reliability and CONTINUE_MISSION.\n")


def test_2_low_battery_reserve():
    print("[TEST 2] Testing Low Battery Reserve Violation (<20% Policy Limit)...")
    engine = MissionReliabilityEngine()
    current_pos = GeographicPosition(18.8000, 74.2000, 3000.0)
    home_pos = GeographicPosition(18.5822, 73.9197, 592.0)

    rel = engine.evaluate(
        timestamp_s=500.0,
        current_pos=current_pos,
        home_pos=home_pos,
        altitude_m=3000.0,
        ground_speed_mps=55.0,
        engine_health_pct=95.0,
        engine_power_avail_w=140000.0,
        fuel_flow_lph=22.0,
        battery_energy_wh=2000.0,
        battery_soc_pct=14.5,       # < 20% limit
        battery_soh_pct=98.0,
        battery_temp_c=32.0,
        battery_is_constrained=False,
        electric_motor_power_avail_w=40000.0,
        current_power_demand_w=55000.0,
        power_deficit_w=0.0,
        mission_progress_pct=40.0,
        quality_status="HEALTHY"
    )

    print(f"  -> Battery SoC: {14.5}% (Policy Min: {engine.config.min_battery_reserve_pct}%)")
    print(f"  -> Contingency Decision: {rel.contingency_recommendation}")
    print(f"  -> Reason: {rel.contingency_reason}")
    print(f"  -> Energy Reserve Factor: {rel.score_breakdown.energy_reserve_factor:.1f}")

    assert rel.contingency_recommendation == ContingencyAction.ABORT_AND_RTB.value
    assert "battery" in rel.contingency_reason.lower()
    assert rel.score_breakdown.energy_reserve_factor < 60.0
    print("  -> PASSED: Battery reserve violation safely triggers ABORT_AND_RTB.\n")


def test_3_low_fuel_reserve():
    print("[TEST 3] Testing Low Fuel Reserve Violation (<18 L Policy Limit)...")
    engine = MissionReliabilityEngine()
    # Force remaining fuel to 14.0 L
    engine.remaining_fuel_liters = 14.0
    current_pos = GeographicPosition(18.8000, 74.2000, 3000.0)
    home_pos = GeographicPosition(18.5822, 73.9197, 592.0)

    rel = engine.evaluate(
        timestamp_s=600.0,
        current_pos=current_pos,
        home_pos=home_pos,
        altitude_m=3000.0,
        ground_speed_mps=55.0,
        engine_health_pct=95.0,
        engine_power_avail_w=140000.0,
        fuel_flow_lph=22.0,
        battery_energy_wh=18000.0,
        battery_soc_pct=85.0,
        battery_soh_pct=98.0,
        battery_temp_c=30.0,
        battery_is_constrained=False,
        electric_motor_power_avail_w=40000.0,
        current_power_demand_w=55000.0,
        power_deficit_w=0.0,
        mission_progress_pct=50.0,
        quality_status="HEALTHY"
    )

    print(f"  -> Fuel Remaining: {rel.available_fuel_liters:.1f} L (Policy Min: {engine.config.min_fuel_reserve_liters} L)")
    print(f"  -> Contingency Decision: {rel.contingency_recommendation}")
    print(f"  -> Reason: {rel.contingency_reason}")

    assert rel.contingency_recommendation == ContingencyAction.ABORT_AND_RTB.value
    assert "fuel" in rel.contingency_reason.lower()
    assert rel.available_fuel_liters < engine.config.min_fuel_reserve_liters
    print("  -> PASSED: Fuel reserve violation safely triggers ABORT_AND_RTB.\n")


def test_4_degraded_engine_health():
    print("[TEST 4] Testing Degraded Engine Mechanical Health (<65%)...")
    engine = MissionReliabilityEngine()
    current_pos = GeographicPosition(18.8000, 74.2000, 3000.0)
    home_pos = GeographicPosition(18.5822, 73.9197, 592.0)

    rel = engine.evaluate(
        timestamp_s=700.0,
        current_pos=current_pos,
        home_pos=home_pos,
        altitude_m=3000.0,
        ground_speed_mps=55.0,
        engine_health_pct=52.0,       # Degraded mechanical health
        engine_power_avail_w=100000.0,
        fuel_flow_lph=24.0,
        battery_energy_wh=18000.0,
        battery_soc_pct=90.0,
        battery_soh_pct=98.0,
        battery_temp_c=30.0,
        battery_is_constrained=False,
        electric_motor_power_avail_w=40000.0,
        current_power_demand_w=55000.0,
        power_deficit_w=0.0,
        mission_progress_pct=35.0,
        quality_status="HEALTHY"
    )

    print(f"  -> Engine Health: {52.0}%")
    print(f"  -> Contingency Decision: {rel.contingency_recommendation}")
    print(f"  -> Propulsion Health Factor: {rel.score_breakdown.propulsion_health_factor:.1f}")
    print(f"  -> Reliability Status: {rel.reliability_status}")

    assert rel.contingency_recommendation == ContingencyAction.ABORT_AND_RTB.value
    assert "engine" in rel.contingency_reason.lower()
    assert rel.score_breakdown.propulsion_health_factor < 60.0
    assert rel.reliability_status in [ReliabilityStatusLevel.CAUTION.value, ReliabilityStatusLevel.CRITICAL.value]
    print("  -> PASSED: Degraded engine health triggers ABORT_AND_RTB.\n")


def test_5_insufficient_return_energy():
    print("[TEST 5] Testing Insufficient Return Energy (RTB Infeasible -> Divert)...")
    engine = MissionReliabilityEngine()
    # UAV 400 km distant from home base with depleted fuel and minimal battery
    engine.remaining_fuel_liters = 5.0
    current_pos = GeographicPosition(21.5000, 76.5000, 6000.0)
    home_pos = GeographicPosition(18.5822, 73.9197, 592.0)

    rel = engine.evaluate(
        timestamp_s=800.0,
        current_pos=current_pos,
        home_pos=home_pos,
        altitude_m=6000.0,
        ground_speed_mps=55.0,
        engine_health_pct=90.0,
        engine_power_avail_w=120000.0,
        fuel_flow_lph=22.0,
        battery_energy_wh=3000.0,
        battery_soc_pct=25.0,
        battery_soh_pct=95.0,
        battery_temp_c=30.0,
        battery_is_constrained=False,
        electric_motor_power_avail_w=40000.0,
        current_power_demand_w=55000.0,
        power_deficit_w=0.0,
        mission_progress_pct=60.0,
        quality_status="HEALTHY"
    )

    print(f"  -> Distance to Home: {rel.rtb_prediction.distance_to_home_m/1000.0:.1f} km")
    print(f"  -> RTB Feasible:     {rel.rtb_feasible}")
    print(f"  -> Contingency:      {rel.contingency_recommendation}")
    print(f"  -> Reason:           {rel.contingency_reason}")

    assert rel.rtb_feasible is False
    assert rel.contingency_recommendation in [
        ContingencyAction.DIVERT_TO_ALTERNATE.value,
        ContingencyAction.EMERGENCY_DESCENT.value
    ]
    print("  -> PASSED: Infeasible RTB correctly redirects to alternate recovery or emergency descent.\n")


def test_6_invalid_telemetry_injection():
    print("[TEST 6] Testing Invalid Telemetry (NaN / Infinite Corruption)...")
    engine = MissionReliabilityEngine()
    current_pos = GeographicPosition(18.8000, 74.2000, 3000.0)
    home_pos = GeographicPosition(18.5822, 73.9197, 592.0)

    rel = engine.evaluate(
        timestamp_s=900.0,
        current_pos=current_pos,
        home_pos=home_pos,
        altitude_m=3000.0,
        ground_speed_mps=55.0,
        engine_health_pct=100.0,
        engine_power_avail_w=140000.0,
        fuel_flow_lph=22.0,
        battery_energy_wh=float('nan'),  # Injected NaN
        battery_soc_pct=float('nan'),     # Injected NaN
        battery_soh_pct=100.0,
        battery_temp_c=25.0,
        battery_is_constrained=False,
        electric_motor_power_avail_w=40000.0,
        current_power_demand_w=55000.0,
        power_deficit_w=0.0,
        mission_progress_pct=25.0,
        quality_status="INVALID"          # Flagged by validator
    )

    print(f"  -> Quality Status:       INVALID")
    print(f"  -> Reliability Index:    {rel.mission_reliability_index:.1f}% (Clamped <= 10.0)")
    print(f"  -> Contingency Decision: {rel.contingency_recommendation}")
    print(f"  -> Telemetry Factor:     {rel.score_breakdown.telemetry_quality_factor:.1f}")

    assert rel.mission_reliability_index <= 10.0
    assert rel.contingency_recommendation != ContingencyAction.CONTINUE_MISSION.value
    assert rel.score_breakdown.telemetry_quality_factor == 0.0
    assert rel.reliability_status == ReliabilityStatusLevel.CRITICAL.value
    print("  -> PASSED: Corrupted telemetry safely penalizes reliability index and prevents continuation.\n")


def test_7_high_propulsion_power_demand():
    print("[TEST 7] Testing High Propulsion Power Demand & Deficit (>8 kW)...")
    engine = MissionReliabilityEngine()
    current_pos = GeographicPosition(18.8000, 74.2000, 3000.0)
    home_pos = GeographicPosition(18.5822, 73.9197, 592.0)

    rel = engine.evaluate(
        timestamp_s=1000.0,
        current_pos=current_pos,
        home_pos=home_pos,
        altitude_m=3000.0,
        ground_speed_mps=55.0,
        engine_health_pct=95.0,
        engine_power_avail_w=100000.0,
        fuel_flow_lph=28.0,
        battery_energy_wh=18000.0,
        battery_soc_pct=90.0,
        battery_soh_pct=98.0,
        battery_temp_c=35.0,
        battery_is_constrained=False,
        electric_motor_power_avail_w=40000.0,
        current_power_demand_w=180000.0,
        power_deficit_w=40000.0,        # 40 kW power deficit (> 8 kW)
        mission_progress_pct=30.0,
        quality_status="HEALTHY"
    )

    print(f"  -> Power Deficit:        {40.0} kW")
    print(f"  -> Power Headroom Factor: {rel.score_breakdown.power_headroom_factor:.1f}")
    print(f"  -> Contingency Decision: {rel.contingency_recommendation}")
    print(f"  -> Reason:               {rel.contingency_reason}")

    assert rel.score_breakdown.power_headroom_factor < 40.0
    assert rel.contingency_recommendation == ContingencyAction.ABORT_AND_RTB.value
    assert "deficit" in rel.contingency_reason.lower()
    print("  -> PASSED: Power deficit degrades power headroom factor and triggers ABORT_AND_RTB.\n")


def test_8_zero_low_ground_speed():
    print("[TEST 8] Testing Zero / Low Ground Speed Fallback...")
    engine = MissionReliabilityEngine()
    current_pos = GeographicPosition(18.8000, 74.2000, 3000.0)
    home_pos = GeographicPosition(18.5822, 73.9197, 592.0)

    # 1. Zero speed
    rtb_zero = engine.predict_rtb_energy(
        current_pos=current_pos,
        home_pos=home_pos,
        current_altitude_m=3000.0,
        ground_speed_mps=0.0,           # Zero speed test
        battery_usable_wh=18000.0,
        battery_soc_pct=90.0,
        remaining_fuel_liters=100.0
    )
    print(f"  -> Zero Speed RTB Duration: {rtb_zero.estimated_duration_s:.1f} s")
    print(f"  -> Estimated Energy:        {rtb_zero.estimated_energy_wh:.1f} Wh")
    assert rtb_zero.estimated_duration_s > 0.0
    assert not math.isnan(rtb_zero.estimated_duration_s)
    assert not math.isinf(rtb_zero.estimated_duration_s)

    # 2. Low speed (0.5 m/s)
    rtb_low = engine.predict_rtb_energy(
        current_pos=current_pos,
        home_pos=home_pos,
        current_altitude_m=3000.0,
        ground_speed_mps=0.5,           # Unphysically low speed
        battery_usable_wh=18000.0,
        battery_soc_pct=90.0,
        remaining_fuel_liters=100.0
    )
    print(f"  -> Low Speed (0.5m/s) Fallback Duration: {rtb_low.estimated_duration_s:.1f} s")
    assert rtb_low.estimated_duration_s > 0.0
    assert not math.isnan(rtb_low.estimated_duration_s)
    print("  -> PASSED: Zero and low speeds safely handled with nominal cruise velocity fallbacks.\n")


def test_9_emergency_contingency_actions():
    print("[TEST 9] Testing Emergency Contingency (Dual Propulsion Loss / Overheat)...")
    engine = MissionReliabilityEngine()
    current_pos = GeographicPosition(18.8000, 74.2000, 3000.0)
    home_pos = GeographicPosition(18.5822, 73.9197, 592.0)

    # Case A: Dual propulsion loss (Engine health < 20% and Battery depleted < 10%)
    rel_dual = engine.evaluate(
        timestamp_s=1100.0,
        current_pos=current_pos,
        home_pos=home_pos,
        altitude_m=3000.0,
        ground_speed_mps=55.0,
        engine_health_pct=12.0,       # Dual failure: Engine failed
        engine_power_avail_w=5000.0,
        fuel_flow_lph=2.0,
        battery_energy_wh=500.0,
        battery_soc_pct=5.0,          # Dual failure: Battery exhausted
        battery_soh_pct=90.0,
        battery_temp_c=45.0,
        battery_is_constrained=True,
        electric_motor_power_avail_w=2000.0,
        current_power_demand_w=55000.0,
        power_deficit_w=48000.0,
        mission_progress_pct=50.0,
        quality_status="HEALTHY"
    )
    print(f"  -> Case A Decision: {rel_dual.contingency_recommendation}")
    print(f"  -> Reason:          {rel_dual.contingency_reason}")
    assert rel_dual.contingency_recommendation == ContingencyAction.EMERGENCY_DESCENT.value
    assert rel_dual.reliability_status == ReliabilityStatusLevel.CRITICAL.value

    # Case B: Battery thermal runaway / critical temperature >= 65 C
    rel_thermal = engine.evaluate(
        timestamp_s=1105.0,
        current_pos=current_pos,
        home_pos=home_pos,
        altitude_m=3000.0,
        ground_speed_mps=55.0,
        engine_health_pct=95.0,
        engine_power_avail_w=140000.0,
        fuel_flow_lph=22.0,
        battery_energy_wh=18000.0,
        battery_soc_pct=85.0,
        battery_soh_pct=95.0,
        battery_temp_c=68.5,          # Thermal critical >= 65 C
        battery_is_constrained=False,
        electric_motor_power_avail_w=40000.0,
        current_power_demand_w=55000.0,
        power_deficit_w=0.0,
        mission_progress_pct=50.0,
        quality_status="HEALTHY"
    )
    print(f"  -> Case B Decision: {rel_thermal.contingency_recommendation}")
    print(f"  -> Reason:          {rel_thermal.contingency_reason}")
    assert rel_thermal.contingency_recommendation == ContingencyAction.EMERGENCY_DESCENT.value
    assert "thermal" in rel_thermal.contingency_reason.lower()
    print("  -> PASSED: Critical emergencies deterministically trigger EMERGENCY_DESCENT.\n")


def test_10_missing_stale_telemetry():
    print("[TEST 10] Testing Missing / Stale Telemetry Lag (>200ms)...")
    powertrain = HybridPowertrain()
    digital_twin = UAVDigitalTwin()
    flight = get_standard_flight_state(15000.0)

    # Initial update
    ht = powertrain.update(50000.0, flight, 15.0, 0.05)
    snapshot = digital_twin.update_from_hybrid_telemetry(ht, flight)

    # Inject 1.0s staleness into mission subsystem
    uav_state = digital_twin.latest_state
    uav_state.mission.timestamp_s = uav_state.timestamp_s - 1.0

    status = UAVDigitalTwinValidator.validate(uav_state, stale_timeout_s=0.20)
    print(f"  -> Validator Status: {status}")
    print(f"  -> Stale Flags:      {uav_state.stale_data_flags}")

    # Evaluate reliability with DEGRADED status
    rel = digital_twin.reliability_engine.evaluate(
        timestamp_s=uav_state.timestamp_s,
        current_pos=uav_state.mission.current_position,
        home_pos=digital_twin.navigation_engine.home_position,
        altitude_m=flight.altitude_m,
        ground_speed_mps=flight.airspeed_mps,
        engine_health_pct=100.0,
        engine_power_avail_w=140000.0,
        fuel_flow_lph=22.0,
        battery_energy_wh=18000.0,
        battery_soc_pct=90.0,
        battery_soh_pct=100.0,
        battery_temp_c=25.0,
        battery_is_constrained=False,
        electric_motor_power_avail_w=40000.0,
        current_power_demand_w=50000.0,
        power_deficit_w=0.0,
        mission_progress_pct=20.0,
        quality_status=status.value,
        stale_data_flags=uav_state.stale_data_flags
    )

    print(f"  -> Telemetry Quality Factor: {rel.score_breakdown.telemetry_quality_factor:.1f}")
    print(f"  -> Contingency Recommendation: {rel.contingency_recommendation}")
    print(f"  -> Reason: {rel.contingency_reason}")

    assert status == DigitalTwinQualityStatus.DEGRADED
    assert uav_state.stale_data_flags["mission"] is True
    assert rel.score_breakdown.telemetry_quality_factor < 60.0
    assert rel.contingency_recommendation == ContingencyAction.ABORT_AND_RTB.value
    assert "degraded" in rel.contingency_reason.lower() or "stale" in rel.contingency_reason.lower()
    print("  -> PASSED: Stale telemetry penalizes telemetry factor and mandates ABORT_AND_RTB.\n")


def run_all_tests():
    print("=" * 80)
    print("  DRDO MALE UAV MISSION RELIABILITY & CONTINGENCY ANALYSIS SUITE (STEP 7)")
    print("=" * 80)
    test_1_normal_healthy_mission()
    test_2_low_battery_reserve()
    test_3_low_fuel_reserve()
    test_4_degraded_engine_health()
    test_5_insufficient_return_energy()
    test_6_invalid_telemetry_injection()
    test_7_high_propulsion_power_demand()
    test_8_zero_low_ground_speed()
    test_9_emergency_contingency_actions()
    test_10_missing_stale_telemetry()
    print("=" * 80)
    print("  ALL 10 MISSION RELIABILITY & CONTINGENCY TESTS PASSED SUCCESSFULLY! (10/10 PASS)")
    print("=" * 80)


if __name__ == "__main__":
    run_all_tests()
