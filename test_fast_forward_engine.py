"""
DRDO MALE UAV Multi-Scenario Predictive Simulation (Step 9)
Root Test Runner Proxy: Fast-Forward Simulation Engine
"""
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from tests.test_fast_forward_engine import (
    test_1_model_state_isolation,
    test_2_battery_depletion_and_reserve_breach,
    test_3_thermal_limit_breach_detection,
    test_4_fuel_reserve_violation_detection,
    test_5_faster_than_real_time_speed_benchmark,
)

if __name__ == "__main__":
    print("=" * 80)
    print("  FAST-FORWARD SIMULATION ENGINE TEST SUITE (STEP 9)")
    print("=" * 80)
    test_1_model_state_isolation()
    print()
    test_2_battery_depletion_and_reserve_breach()
    print()
    test_3_thermal_limit_breach_detection()
    print()
    test_4_fuel_reserve_violation_detection()
    print()
    test_5_faster_than_real_time_speed_benchmark()
    print()
    print("=" * 80)
    print("  ALL 5 FAST-FORWARD ENGINE TESTS PASSED! (5/5 PASS)")
    print("=" * 80)
