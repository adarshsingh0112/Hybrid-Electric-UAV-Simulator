"""
DRDO MALE UAV AI-Guided Mission Decision Engine Test Suite (Step 8)
Root test runner proxy.
"""
from pathlib import Path
import sys

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

# Run tests from tests/test_mission_decision_engine.py
from tests.test_mission_decision_engine import (
    test_1_normal_healthy_sortie,
    test_2_low_battery_reserve_violation,
    test_3_critically_depleted_battery,
    test_4_low_fuel_reserve_violation,
    test_5_degraded_engine_mechanical_health,
    test_6_insufficient_return_energy_to_home,
    test_7_dual_propulsion_loss_emergency,
    test_8_critical_battery_overheat,
    test_9_invalid_corrupted_telemetry,
    test_10_stale_telemetry_lag,
    test_11_explicit_missing_critical_telemetry,
    test_12_config_validation_and_safety_checks,
)

if __name__ == "__main__":
    print("=" * 80)
    print("  DRDO MALE UAV AI MISSION DECISION ENGINE TEST SUITE (STEP 8)")
    print("=" * 80)

    test_1_normal_healthy_sortie()
    print()
    test_2_low_battery_reserve_violation()
    print()
    test_3_critically_depleted_battery()
    print()
    test_4_low_fuel_reserve_violation()
    print()
    test_5_degraded_engine_mechanical_health()
    print()
    test_6_insufficient_return_energy_to_home()
    print()
    test_7_dual_propulsion_loss_emergency()
    print()
    test_8_critical_battery_overheat()
    print()
    test_9_invalid_corrupted_telemetry()
    print()
    test_10_stale_telemetry_lag()
    print()
    test_11_explicit_missing_critical_telemetry()
    print()
    test_12_config_validation_and_safety_checks()
    print()

    print("=" * 80)
    print("  ALL 12 MISSION DECISION ENGINE TESTS PASSED SUCCESSFULLY! (12/12 PASS)")
    print("=" * 80)
