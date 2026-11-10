"""
Hybrid-Electric Powertrain Integration Test Suite
Tests:
1. ENGINE_ONLY operating mode (engine meets propulsion demand, battery idle).
2. ELECTRIC_ONLY operating mode (engine idle, electric motor powers propulsion, battery SoC decreases).
3. MOTOR_ASSIST operating mode (engine + electric motor parallel drive, battery discharges).
4. BATTERY_CHARGING operating mode (engine supplies propulsion + generator charge power, battery SoC increases).
5. LOW BATTERY SOC constraint (electric power constrained, power deficit exposed, no false claims).
6. HIGH POWER DEMAND & POWER CONSERVATION (power balance respected, actual propulsion bounded by physical limits).
"""

import sys
from pathlib import Path
import math

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from simulation.flight_profile import FlightProfile, FlightState
from simulation.mvem import MeanValueEngineModel
from simulation.battery import BatteryModel
from simulation.hybrid_powertrain import (
    HybridPowertrain,
    HybridPowertrainConfig,
    HybridOperatingMode,
    HybridControlMode,
    PowerManagementController,
)

def get_test_flight_state(altitude_ft: float = 10000.0) -> FlightState:
    """Helper creating standard steady-state flight condition."""
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

def test_engine_only_mode():
    print("[TEST 1] Testing ENGINE_ONLY Mode...")
    powertrain = HybridPowertrain()
    flight = get_test_flight_state(altitude_ft=5000.0)
    
    initial_soc = powertrain.battery.soc_pct
    req_prop_w = 80000.0 # 80 kW demand
    
    # Run simulation for 60 steps (3 seconds)
    telemetry = None
    for _ in range(60):
        telemetry = powertrain.update(
            propulsion_power_request_w=req_prop_w,
            flight_state=flight,
            ambient_temp_c=15.0,
            dt_s=0.05,
            mode_override=HybridOperatingMode.ENGINE_ONLY
        )

    print(f"  -> Requested Propulsion Power: {req_prop_w/1000:.1f} kW | Actual Delivered: {telemetry.propulsion_power_actual_w/1000:.1f} kW")
    print(f"  -> Engine Power Contribution:  {telemetry.engine_power_contribution_w/1000:.1f} kW ({telemetry.engine_power_contribution_hp:.1f} HP)")
    print(f"  -> Motor Power Contribution:   {telemetry.motor_power_contribution_w/1000:.1f} kW")
    print(f"  -> Battery Current:            {telemetry.battery_current_a:.2f} A | Final SoC: {telemetry.battery_soc_pct:.2f}%")

    assert telemetry.operating_mode == HybridOperatingMode.ENGINE_ONLY.value
    assert telemetry.engine_power_contribution_w > 0.90 * req_prop_w, "Engine did not supply primary propulsion power"
    assert abs(telemetry.motor_power_contribution_w) < 1.0, "Motor should not contribute in ENGINE_ONLY mode"
    assert abs(telemetry.battery_current_a) < 0.1, "Battery current should be zero in ENGINE_ONLY mode"
    assert abs(telemetry.battery_soc_pct - initial_soc) < 0.01, "Battery SoC should remain unchanged"
    print("  -> PASSED: ENGINE_ONLY mode verified (engine meets demand, battery remains idle).")


def test_electric_only_mode():
    print("\n[TEST 2] Testing ELECTRIC_ONLY Mode...")
    powertrain = HybridPowertrain()
    flight = get_test_flight_state(altitude_ft=5000.0)
    
    initial_soc = powertrain.battery.soc_pct
    req_prop_w = 25000.0 # 25 kW loiter demand
    
    # Run simulation for 1200 steps (60 seconds)
    telemetry = None
    for _ in range(1200):
        telemetry = powertrain.update(
            propulsion_power_request_w=req_prop_w,
            flight_state=flight,
            ambient_temp_c=15.0,
            dt_s=0.05,
            mode_override=HybridOperatingMode.ELECTRIC_ONLY
        )

    print(f"  -> Requested Propulsion Power: {req_prop_w/1000:.1f} kW | Actual Delivered: {telemetry.propulsion_power_actual_w/1000:.1f} kW")
    print(f"  -> Engine Power Contribution:  {telemetry.engine_power_contribution_w/1000:.1f} kW (Throttle: {telemetry.engine_throttle_pct:.1f}%)")
    print(f"  -> Motor Power Contribution:   {telemetry.motor_power_contribution_w/1000:.1f} kW")
    print(f"  -> Initial SoC: {initial_soc:.2f}% | Final SoC: {telemetry.battery_soc_pct:.2f}% (Dropped by {initial_soc - telemetry.battery_soc_pct:.2f}%)")
    print(f"  -> Battery Power: {telemetry.battery_power_w/1000:.1f} kW | Current: {telemetry.battery_current_a:.1f} A")

    assert telemetry.operating_mode == HybridOperatingMode.ELECTRIC_ONLY.value
    assert telemetry.engine_power_contribution_w == 0.0, "Engine propulsion contribution should be 0.0 W in ELECTRIC_ONLY"
    assert telemetry.motor_power_contribution_w > 0.95 * req_prop_w, "Electric motor did not supply propulsion power"
    assert telemetry.battery_soc_pct < initial_soc, "Battery SoC should decrease in ELECTRIC_ONLY mode"
    assert telemetry.battery_current_a > 0.0, "Battery current should be positive (discharging)"
    print("  -> PASSED: ELECTRIC_ONLY mode verified (engine idle, motor delivers power, battery discharges).")


def test_motor_assist_mode():
    print("\n[TEST 3] Testing MOTOR_ASSIST Mode...")
    powertrain = HybridPowertrain()
    flight = get_test_flight_state(altitude_ft=5000.0)
    
    initial_soc = powertrain.battery.soc_pct
    req_prop_w = 160000.0 # 160 kW high climb demand
    
    # Run simulation for 200 steps (10 seconds)
    telemetry = None
    for _ in range(200):
        telemetry = powertrain.update(
            propulsion_power_request_w=req_prop_w,
            flight_state=flight,
            ambient_temp_c=15.0,
            dt_s=0.05,
            mode_override=HybridOperatingMode.MOTOR_ASSIST
        )

    print(f"  -> Requested Propulsion Power: {req_prop_w/1000:.1f} kW | Actual Delivered: {telemetry.propulsion_power_actual_w/1000:.1f} kW")
    print(f"  -> Engine Power Contribution:  {telemetry.engine_power_contribution_w/1000:.1f} kW ({telemetry.engine_power_contribution_hp:.1f} HP)")
    print(f"  -> Motor Power Contribution:   {telemetry.motor_power_contribution_w/1000:.1f} kW ({telemetry.motor_power_contribution_hp:.1f} HP)")
    print(f"  -> Battery Current: {telemetry.battery_current_a:.1f} A | Final SoC: {telemetry.battery_soc_pct:.2f}%")
    print(f"  -> Energy Flow Direction: {telemetry.energy_flow_direction}")

    assert telemetry.operating_mode == HybridOperatingMode.MOTOR_ASSIST.value
    assert telemetry.engine_power_contribution_w > 100000.0, "Engine should operate at high output"
    assert telemetry.motor_power_contribution_w > 10000.0, "Electric motor should assist with additional power"
    assert telemetry.battery_soc_pct < initial_soc, "Battery SoC should decrease during motor assist"
    assert telemetry.energy_flow_direction == "DUAL_TO_PROPULSION"
    print("  -> PASSED: MOTOR_ASSIST mode verified (parallel engine + motor propulsion drive).")


def test_battery_charging_mode():
    print("\n[TEST 4] Testing BATTERY_CHARGING Mode...")
    # Initialize battery at 50% SoC
    powertrain = HybridPowertrain()
    powertrain.battery.reset(soc_pct=50.0)
    flight = get_test_flight_state(altitude_ft=5000.0)
    
    initial_soc = powertrain.battery.soc_pct
    req_prop_w = 40000.0 # 40 kW cruise propulsion demand
    
    # Run simulation for 1200 steps (60 seconds)
    telemetry = None
    for _ in range(1200):
        telemetry = powertrain.update(
            propulsion_power_request_w=req_prop_w,
            flight_state=flight,
            ambient_temp_c=15.0,
            dt_s=0.05,
            mode_override=HybridOperatingMode.BATTERY_CHARGING
        )

    print(f"  -> Propulsion Power Demand:    {req_prop_w/1000:.1f} kW | Actual Delivered: {telemetry.propulsion_power_actual_w/1000:.1f} kW")
    print(f"  -> Engine Power Contribution:  {telemetry.engine_power_contribution_w/1000:.1f} kW (Throttle: {telemetry.engine_throttle_pct:.1f}%)")
    print(f"  -> Generator Mechanical Drag:  {telemetry.electric_machine_power_w/1000:.1f} kW (Mode: {telemetry.electric_machine_mode})")
    print(f"  -> Initial SoC: {initial_soc:.2f}% | Final SoC: {telemetry.battery_soc_pct:.2f}% (Increased by {telemetry.battery_soc_pct - initial_soc:.2f}%)")
    print(f"  -> Battery Current: {telemetry.battery_current_a:.1f} A (Negative = Charging)")

    assert telemetry.operating_mode == HybridOperatingMode.BATTERY_CHARGING.value
    assert telemetry.propulsion_power_actual_w >= 0.95 * req_prop_w, "Engine should satisfy propulsion demand"
    assert telemetry.electric_machine_mode == "GENERATOR", "Electric machine should operate as GENERATOR"
    assert telemetry.battery_current_a < 0.0, "Battery current should be negative (charging)"
    assert telemetry.battery_soc_pct > initial_soc, "Battery SoC should increase during charging"
    print("  -> PASSED: BATTERY_CHARGING mode verified (engine drives propulsion + generator charges battery).")


def test_low_battery_soc_constraint():
    print("\n[TEST 5] Testing Low Battery SoC Constraint...")
    # Initialize near empty battery (0.2% SoC)
    powertrain = HybridPowertrain()
    powertrain.battery.reset(soc_pct=0.2)
    flight = get_test_flight_state(altitude_ft=5000.0)
    
    req_prop_w = 30000.0 # 30 kW demand
    
    # Request ELECTRIC_ONLY mode on depleted battery
    telemetry = powertrain.update(
        propulsion_power_request_w=req_prop_w,
        flight_state=flight,
        ambient_temp_c=15.0,
        dt_s=0.05,
        mode_override=HybridOperatingMode.ELECTRIC_ONLY
    )

    print(f"  -> Mode Requested: ELECTRIC_ONLY | Effective Mode Selected: {telemetry.operating_mode}")
    print(f"  -> Requested Propulsion Power: {req_prop_w/1000:.1f} kW | Actual Delivered: {telemetry.propulsion_power_actual_w/1000:.1f} kW")
    print(f"  -> Power Deficit: {telemetry.propulsion_power_deficit_w/1000:.1f} kW | System Constrained: {telemetry.is_system_power_constrained}")

    assert telemetry.motor_power_contribution_w == 0.0, "Electric motor should not deliver power when battery is depleted"
    assert telemetry.propulsion_power_deficit_w > 0.0 or telemetry.operating_mode == HybridOperatingMode.ENGINE_ONLY.value, "Power deficit or fallback mode should be triggered"
    assert telemetry.is_system_power_constrained == True, "System power constrained flag should be True"
    print("  -> PASSED: Low Battery SoC constraint verified (no false propulsion power reporting).")


def test_high_power_demand_and_conservation():
    print("\n[TEST 6] Testing High Power Demand & Power Conservation Limits...")
    powertrain = HybridPowertrain()
    flight = get_test_flight_state(altitude_ft=30000.0) # 30k ft critical altitude (engine max ~82 kW)
    
    # Request 300 kW (exceeds engine ~122 kW + motor 40 kW = max ~162 kW)
    req_prop_w = 300000.0
    
    telemetry = powertrain.update(
        propulsion_power_request_w=req_prop_w,
        flight_state=flight,
        ambient_temp_c=-40.0,
        dt_s=0.05,
        mode_override=HybridOperatingMode.MOTOR_ASSIST
    )

    p_eng = telemetry.engine_power_contribution_w
    p_mot = telemetry.motor_power_contribution_w
    p_act = telemetry.propulsion_power_actual_w
    p_def = telemetry.propulsion_power_deficit_w

    print(f"  -> Altitude: 30,000 ft | Requested Power: {req_prop_w/1000:.1f} kW")
    print(f"  -> Engine Output: {p_eng/1000:.1f} kW | Motor Output: {p_mot/1000:.1f} kW")
    print(f"  -> Actual Delivered Propulsion Power: {p_act/1000:.1f} kW")
    print(f"  -> Reported Power Deficit:           {p_def/1000:.1f} kW")
    print(f"  -> Power Balance Check (P_act - P_eng - P_mot): {abs(p_act - (p_eng + p_mot)):.2f} W")
    print(f"  -> System Constrained Flag: {telemetry.is_system_power_constrained}")

    assert abs(p_act - (p_eng + p_mot)) < 1.0, "Power conservation equation violated: P_act != P_eng + P_mot"
    assert p_def > 80000.0, "Power deficit should accurately reflect unmet demand"
    assert telemetry.is_system_power_constrained == True, "System should report constrained state under overload"
    print("  -> PASSED: High power demand & power conservation verified (P_act = P_eng + P_mot bounded by hardware limits).")


if __name__ == "__main__":
    print("=" * 75)
    print("  DRDO MALE UAV HYBRID-ELECTRIC POWERTRAIN TEST SUITE (STEP 3)")
    print("=" * 75)
    test_engine_only_mode()
    test_electric_only_mode()
    test_motor_assist_mode()
    test_battery_charging_mode()
    test_low_battery_soc_constraint()
    test_high_power_demand_and_conservation()
    print("\n" + "=" * 75)
    print("ALL HYBRID-ELECTRIC POWERTRAIN TESTS PASSED SUCCESSFULLY! (6/6 PASS)")
    print("=" * 75)
