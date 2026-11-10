"""
DRDO MALE UAV Multi-Criteria Trajectory Optimizer Test Suite (Step 8)
Root test runner proxy.
"""
from pathlib import Path
import sys

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

# Run tests from tests/test_trajectory_optimizer.py
from tests.test_trajectory_optimizer import (
    test_1_candidate_routes_generation,
    test_2_aerodynamic_power_and_hybrid_strategy,
    test_3_hard_safety_reserve_constraints,
    test_4_multi_criteria_cost_function,
    test_5_dynamic_candidate_ranking,
    test_6_emergency_scenario_no_feasible_route,
    test_7_execution_throttling_and_caching,
)

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
