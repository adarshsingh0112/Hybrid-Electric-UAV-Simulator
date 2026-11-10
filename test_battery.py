"""
Battery Energy Storage System (BESS) Verification Test Suite
Tests:
1. Idle battery state (zero power demand, minimal SoC change).
2. Moderate discharge (gradual SoC decay and realistic thermal rise).
3. High power demand (higher current, I^2 * R Joule losses, accelerated thermal rise).
4. Battery charging (SoC increases without exceeding 100% boundary).
5. Low SoC / depleted battery boundary (safe power constraint enforcement).
"""

import sys
from pathlib import Path
import math
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from simulation.battery import BatteryModel, BatteryConfig, BatteryChargeState

def test_idle_battery():
    print("[TEST 1] Testing Idle Battery State (Zero Power Demand)...")
    battery = BatteryModel(initial_soc_pct=80.0, initial_temp_c=25.0)
    
    initial_soc = battery.soc_pct
    initial_temp = battery.temperature_c
    
    # Run idle for 120 seconds (dt = 0.05s, 2400 steps)
    for _ in range(2400):
        telemetry = battery.update(power_request=0.0, ambient_temperature=25.0, dt=0.05)

    print(f"  -> Initial SoC: {initial_soc:.2f}% | Final SoC: {telemetry.soc_pct:.2f}%")
    print(f"  -> Initial Temp: {initial_temp:.2f} °C | Final Temp: {telemetry.temperature_c:.2f} °C")
    print(f"  -> Current: {telemetry.current_a:.2f} A | Charge State: {telemetry.charge_state}")

    assert abs(telemetry.soc_pct - initial_soc) < 0.001, "Idle battery SoC changed unexpectedly"
    assert abs(telemetry.temperature_c - initial_temp) < 0.1, "Idle battery temperature drifted unexpectedly"
    assert telemetry.current_a == 0.0, "Idle battery current should be 0.0 A"
    assert telemetry.charge_state == BatteryChargeState.IDLE.value, f"Expected IDLE state, got {telemetry.charge_state}"
    print("  -> PASSED: Idle battery state verified (SoC and temperature remain stable).")


def test_moderate_discharge():
    print("\n[TEST 2] Testing Moderate Discharge (15 kW Load)...")
    battery = BatteryModel(initial_soc_pct=90.0, initial_temp_c=25.0)
    
    initial_soc = battery.soc_pct
    initial_temp = battery.temperature_c
    
    # Run 15 kW discharge for 300 seconds (5 minutes)
    mod_telemetry = None
    for _ in range(6000):
        mod_telemetry = battery.step(power_request_w=15000.0, ambient_temp_c=25.0, dt_s=0.05)

    print(f"  -> Initial SoC: {initial_soc:.2f}% | Final SoC: {mod_telemetry.soc_pct:.2f}% (Dropped by {initial_soc - mod_telemetry.soc_pct:.2f}%)")
    print(f"  -> Initial Temp: {initial_temp:.2f} °C | Final Temp: {mod_telemetry.temperature_c:.2f} °C (Rose by {mod_telemetry.temperature_c - initial_temp:.2f} °C)")
    print(f"  -> Terminal Voltage: {mod_telemetry.terminal_voltage_v:.2f} V | Current: {mod_telemetry.current_a:.2f} A")
    print(f"  -> Power Loss (I^2 * R): {mod_telemetry.power_loss_w:.2f} W | State: {mod_telemetry.charge_state}")

    assert mod_telemetry.soc_pct < initial_soc, "SoC did not decrease during moderate discharge"
    assert mod_telemetry.temperature_c > initial_temp, "Temperature did not increase during moderate discharge"
    assert mod_telemetry.current_a > 0.0, "Discharge current should be positive"
    assert mod_telemetry.charge_state == BatteryChargeState.DISCHARGING.value, "Expected DISCHARGING state"
    print("  -> PASSED: Moderate discharge verified (gradual SoC decay and realistic thermal rise).")


def test_high_power_demand():
    print("\n[TEST 3] Testing High Power Demand (45 kW Load vs Moderate Load)...")
    battery_high = BatteryModel(initial_soc_pct=90.0, initial_temp_c=25.0)
    
    # Run 45 kW discharge for 60 seconds
    high_telemetry = None
    for _ in range(1200):
        high_telemetry = battery_high.step(power_request_w=45000.0, ambient_temp_c=25.0, dt_s=0.05)

    battery_mod = BatteryModel(initial_soc_pct=90.0, initial_temp_c=25.0)
    mod_telemetry = None
    for _ in range(1200):
        mod_telemetry = battery_mod.step(power_request_w=15000.0, ambient_temp_c=25.0, dt_s=0.05)

    print(f"  -> 15 kW Load Current: {mod_telemetry.current_a:.2f} A | 45 kW Load Current: {high_telemetry.current_a:.2f} A")
    print(f"  -> 15 kW Power Loss:   {mod_telemetry.power_loss_w:.2f} W | 45 kW Power Loss:   {high_telemetry.power_loss_w:.2f} W")
    print(f"  -> 15 kW Temp Rise:   +{mod_telemetry.temperature_c - 25.0:.2f} °C | 45 kW Temp Rise:   +{high_telemetry.temperature_c - 25.0:.2f} °C")

    assert high_telemetry.current_a > mod_telemetry.current_a * 2.5, "High power demand did not draw expected high current"
    assert high_telemetry.power_loss_w > mod_telemetry.power_loss_w * 5.0, "I^2*R losses did not scale quadratically"
    assert high_telemetry.temperature_c > mod_telemetry.temperature_c, "High power demand temperature rise was not higher"
    print("  -> PASSED: High power demand verified (higher current, quadratic Joule losses, faster heating).")


def test_battery_charging():
    print("\n[TEST 4] Testing Battery Charging (-15 kW Charge Demand)...")
    battery = BatteryModel(initial_soc_pct=98.5, initial_temp_c=25.0)
    
    initial_soc = battery.soc_pct
    
    # Charge for 300 seconds (-15 kW charge request)
    chg_telemetry = None
    for _ in range(6000):
        chg_telemetry = battery.update(power_request=-15000.0, ambient_temperature=25.0, dt=0.05)

    print(f"  -> Initial SoC: {initial_soc:.2f}% | Final SoC: {chg_telemetry.soc_pct:.2f}%")
    print(f"  -> Terminal Voltage: {chg_telemetry.terminal_voltage_v:.2f} V | Current: {chg_telemetry.current_a:.2f} A")
    print(f"  -> Charge State: {chg_telemetry.charge_state} | Power Constrained: {chg_telemetry.is_power_constrained}")

    assert chg_telemetry.soc_pct > initial_soc, "SoC did not increase during charging"
    assert chg_telemetry.soc_pct <= 100.0, f"SoC exceeded 100% upper boundary: {chg_telemetry.soc_pct}%"
    assert chg_telemetry.current_a <= 0.0, "Charge current should be negative or zero"
    print("  -> PASSED: Battery charging verified (SoC increases without exceeding 100% limit).")


def test_low_soc_constraint():
    print("\n[TEST 5] Testing Low SoC Boundary & Power Availability Constraint...")
    # Initialize near empty (0.2% SoC)
    battery = BatteryModel(initial_soc_pct=0.2, initial_temp_c=25.0)
    
    # Attempt high discharge demand (30 kW)
    low_telemetry = battery.update(power_request=30000.0, ambient_temperature=25.0, dt=0.05)

    print(f"  -> Requested Power: 30,000 W | Actual Delivered Power: {low_telemetry.power_actual_w:.1f} W")
    print(f"  -> Current SoC: {low_telemetry.soc_pct:.2f}% | Charge State: {low_telemetry.charge_state}")
    print(f"  -> Is Power Constrained: {low_telemetry.is_power_constrained}")

    assert low_telemetry.is_power_constrained == True, "Power should be constrained at depleted SoC"
    assert low_telemetry.power_actual_w == 0.0, f"Depleted battery should deliver 0 W, got {low_telemetry.power_actual_w} W"
    assert low_telemetry.soc_pct >= 0.0, "SoC dropped below 0.0%"
    assert low_telemetry.charge_state in [BatteryChargeState.DEPLETED.value, BatteryChargeState.IDLE.value]
    print("  -> PASSED: Low SoC constraint verified (discharge power safely limited at empty boundary).")


if __name__ == "__main__":
    print("=" * 75)
    print("  DRDO MALE UAV BATTERY ENERGY STORAGE SYSTEM (BESS) TEST SUITE")
    print("=" * 75)
    test_idle_battery()
    test_moderate_discharge()
    test_high_power_demand()
    test_battery_charging()
    test_low_soc_constraint()
    print("\n" + "=" * 75)
    print("ALL BESS BATTERY MODEL TESTS PASSED SUCCESSFULLY! (5/5 PASS)")
    print("=" * 75)
