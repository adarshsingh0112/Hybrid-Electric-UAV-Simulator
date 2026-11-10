"""
Unified Real-Time UAV Digital Twin State Layer Test Suite (Step 4)
Tests:
1. Normal synchronized hybrid operation (all subsystems synchronized, status = HEALTHY).
2. Battery constrained scenario (power constraint propagates correctly, Digital Twin status valid).
3. High propulsion demand with power deficit (deficit represented, validation passes for consistent deficit).
4. Invalid subsystem value (NaN / impossible value injection -> status = INVALID / errors caught).
5. Power consistency violation (artificially corrupted power sum -> consistency error detected).
6. Bounded history buffer (chronological ordering preserved, memory bounded).
"""

import sys
from pathlib import Path
import math

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from simulation.flight_profile import FlightProfile, FlightState
from simulation.hybrid_powertrain import HybridPowertrain, HybridOperatingMode
from digital_twin.uav_digital_twin import (
    UAVDigitalTwin,
    UAVDigitalTwinState,
    UAVDigitalTwinTelemetry,
    DigitalTwinQualityStatus,
    UAVDigitalTwinValidator,
    UAVDigitalTwinHistoryBuffer,
)

def get_test_flight_state(altitude_ft: float = 10000.0) -> FlightState:
    alt_m = FlightProfile.feet_to_meters(altitude_ft)
    t_c, p_bar, rho = FlightProfile.get_isa_atmosphere(alt_m)
    return FlightState(
        time_s=100.0,
        altitude_ft=altitude_ft,
        altitude_m=alt_m,
        throttle_pct=70.0,
        airspeed_mps=55.0,
        ambient_temp_c=t_c,
        ambient_pressure_bar=p_bar,
        air_density_kgpm3=rho,
        phase="HIGH_ALT_LOITER"
    )

def test_normal_synchronized_operation():
    print("[TEST 1] Testing Normal Synchronized Hybrid Operation...")
    powertrain = HybridPowertrain()
    digital_twin = UAVDigitalTwin()
    flight = get_test_flight_state(5000.0)

    # Execute 50 simulation steps
    snapshot = None
    for _ in range(50):
        hybrid_telemetry = powertrain.update(
            propulsion_power_request_w=50000.0,
            flight_state=flight,
            ambient_temp_c=15.0,
            dt_s=0.05,
            mode_override=HybridOperatingMode.ENGINE_ONLY
        )
        snapshot = digital_twin.update_from_hybrid_telemetry(hybrid_telemetry, flight)

    print(f"  -> Master Timestamp: {snapshot.timestamp_s:.2f} s")
    print(f"  -> Digital Twin Quality Status: {snapshot.quality_status}")
    print(f"  -> Validation Warnings: {len(snapshot.validation_warnings)} | Errors: {len(snapshot.validation_errors)}")
    print(f"  -> Subsystem Engine RPM: {snapshot.engine.rpm:.1f} RPM | Battery SoC: {snapshot.battery.soc_pct:.1f}%")

    assert snapshot.quality_status == DigitalTwinQualityStatus.HEALTHY.value
    assert len(snapshot.validation_errors) == 0
    assert snapshot.engine.timestamp_s == snapshot.battery.timestamp_s == snapshot.timestamp_s
    print("  -> PASSED: All subsystems synchronized, Digital Twin status HEALTHY.")


def test_battery_constrained_scenario():
    print("\n[TEST 2] Testing Battery Constrained Scenario...")
    powertrain = HybridPowertrain()
    powertrain.battery.reset(soc_pct=0.2) # Battery depleted
    digital_twin = UAVDigitalTwin()
    flight = get_test_flight_state(5000.0)

    # Request ELECTRIC_ONLY on depleted battery
    hybrid_telemetry = powertrain.update(
        propulsion_power_request_w=30000.0,
        flight_state=flight,
        ambient_temp_c=15.0,
        dt_s=0.05,
        mode_override=HybridOperatingMode.ELECTRIC_ONLY
    )
    snapshot = digital_twin.update_from_hybrid_telemetry(hybrid_telemetry, flight)

    print(f"  -> Battery Constrained Flag: {snapshot.battery.is_power_constrained}")
    print(f"  -> System Constrained Flag:  {snapshot.powertrain.is_system_power_constrained}")
    print(f"  -> Quality Status:           {snapshot.quality_status}")

    assert snapshot.battery.is_power_constrained == True
    assert snapshot.powertrain.is_system_power_constrained == True
    # The Digital Twin representation itself remains valid/trustworthy even when battery is physically constrained
    assert snapshot.quality_status in [DigitalTwinQualityStatus.HEALTHY.value, DigitalTwinQualityStatus.WARNING.value]
    assert len(snapshot.validation_errors) == 0
    print("  -> PASSED: Battery power constraint correctly propagated into Digital Twin state.")


def test_high_demand_power_deficit():
    print("\n[TEST 3] Testing High Propulsion Demand with Power Deficit...")
    powertrain = HybridPowertrain()
    digital_twin = UAVDigitalTwin()
    flight = get_test_flight_state(30000.0) # High altitude derating

    # Request 300 kW demand
    hybrid_telemetry = powertrain.update(
        propulsion_power_request_w=300000.0,
        flight_state=flight,
        ambient_temp_c=-40.0,
        dt_s=0.05,
        mode_override=HybridOperatingMode.MOTOR_ASSIST
    )
    snapshot = digital_twin.update_from_hybrid_telemetry(hybrid_telemetry, flight)

    print(f"  -> Requested Power: {snapshot.powertrain.propulsion_power_req_w/1000:.1f} kW")
    print(f"  -> Actual Power:    {snapshot.powertrain.propulsion_power_actual_w/1000:.1f} kW")
    print(f"  -> Deficit:         {snapshot.powertrain.propulsion_power_deficit_w/1000:.1f} kW")
    print(f"  -> Quality Status:  {snapshot.quality_status}")

    assert snapshot.powertrain.propulsion_power_deficit_w > 80000.0
    assert snapshot.quality_status in [DigitalTwinQualityStatus.HEALTHY.value, DigitalTwinQualityStatus.WARNING.value]
    assert len(snapshot.validation_errors) == 0
    print("  -> PASSED: Power deficit correctly represented and physically validated.")


def test_invalid_subsystem_value():
    print("\n[TEST 4] Testing Invalid Subsystem Value Injection (NaN & Impossible SoC)...")
    powertrain = HybridPowertrain()
    digital_twin = UAVDigitalTwin()
    flight = get_test_flight_state(5000.0)

    hybrid_telemetry = powertrain.update(50000.0, flight, 15.0, 0.05)
    snapshot = digital_twin.update_from_hybrid_telemetry(hybrid_telemetry, flight)
    
    # Artificially inject NaN into state
    uav_state = digital_twin.latest_state
    uav_state.battery.soc_pct = float('nan') # Inject NaN
    
    status = UAVDigitalTwinValidator.validate(uav_state)

    print(f"  -> Quality Status after NaN Injection: {uav_state.quality_status}")
    print(f"  -> Validation Errors Detected: {uav_state.validation_errors}")

    assert uav_state.quality_status == DigitalTwinQualityStatus.INVALID
    assert len(uav_state.validation_errors) > 0
    assert any("NaN" in err or "Invalid" in err for err in uav_state.validation_errors)
    print("  -> PASSED: NaN data corruption caught and marked INVALID.")


def test_power_consistency_violation():
    print("\n[TEST 5] Testing Power Consistency Violation...")
    powertrain = HybridPowertrain()
    digital_twin = UAVDigitalTwin()
    flight = get_test_flight_state(5000.0)

    hybrid_telemetry = powertrain.update(50000.0, flight, 15.0, 0.05)
    snapshot = digital_twin.update_from_hybrid_telemetry(hybrid_telemetry, flight)

    # Corrupt internal power balance equation artificially
    uav_state = digital_twin.latest_state
    uav_state.powertrain.propulsion_power_actual_w += 15000.0 # Add fake 15 kW

    status = UAVDigitalTwinValidator.validate(uav_state)

    print(f"  -> Quality Status after Balance Corruption: {uav_state.quality_status}")
    print(f"  -> Validation Errors Detected: {uav_state.validation_errors}")

    assert uav_state.quality_status == DigitalTwinQualityStatus.INVALID
    assert any("Power conservation violation" in err for err in uav_state.validation_errors)
    print("  -> PASSED: Power balance conservation error detected.")


def test_history_buffer():
    print("\n[TEST 6] Testing Bounded History Buffer...")
    powertrain = HybridPowertrain()
    digital_twin = UAVDigitalTwin(history_buffer_size=50) # Buffer size 50
    flight = get_test_flight_state(5000.0)

    # Execute 150 updates (exceeds buffer capacity)
    for step_idx in range(150):
        hybrid_telemetry = powertrain.update(50000.0, flight, 15.0, 0.05)
        digital_twin.update_from_hybrid_telemetry(hybrid_telemetry, flight)

    buffer_len = len(digital_twin.history_buffer)
    latest_snap = digital_twin.history_buffer.get_latest()
    history = digital_twin.history_buffer.get_history()

    print(f"  -> Total Updates Executed: 150 | Buffer Max Capacity: 50 | Buffer Actual Length: {buffer_len}")
    print(f"  -> Latest Snapshot Timestamp: {latest_snap.timestamp_s:.2f} s")
    print(f"  -> History Chronological Order Check: t[0]={history[0].timestamp_s:.2f}s < t[-1]={history[-1].timestamp_s:.2f}s")

    assert buffer_len == 50, f"History buffer exceeded max capacity: {buffer_len}"
    assert latest_snap.timestamp_s == history[-1].timestamp_s
    assert history[0].timestamp_s < history[-1].timestamp_s, "Chronological order violated"
    print("  -> PASSED: History buffer is bounded, chronologically ordered, and memory safe.")


if __name__ == "__main__":
    print("=" * 75)
    print("  DRDO MALE UAV UNIFIED DIGITAL TWIN STATE LAYER TEST SUITE (STEP 4)")
    print("=" * 75)
    test_normal_synchronized_operation()
    test_battery_constrained_scenario()
    test_high_demand_power_deficit()
    test_invalid_subsystem_value()
    test_power_consistency_violation()
    test_history_buffer()
    print("\n" + "=" * 75)
    print("ALL UNIFIED DIGITAL TWIN TESTS PASSED SUCCESSFULLY! (6/6 PASS)")
    print("=" * 75)
