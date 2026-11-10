"""
DRDO MALE UAV Multi-Scenario Predictive Simulation (Step 9)
Root Test Runner Proxy: Predictive Risk Analyzer
"""
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from tests.test_predictive_risk_analyzer import (
    test_1_statistical_percentiles_and_wilson_ci,
    test_2_route_ensemble_evaluation,
    test_3_multi_route_comparison_identical_scenarios,
    test_4_hard_safety_reserve_precedence,
    test_5_telemetry_snapshot_and_disclaimer,
)

if __name__ == "__main__":
    print("=" * 80)
    print("  PREDICTIVE RISK ANALYZER TEST SUITE (STEP 9)")
    print("=" * 80)
    test_1_statistical_percentiles_and_wilson_ci()
    print()
    test_2_route_ensemble_evaluation()
    print()
    test_3_multi_route_comparison_identical_scenarios()
    print()
    test_4_hard_safety_reserve_precedence()
    print()
    test_5_telemetry_snapshot_and_disclaimer()
    print()
    print("=" * 80)
    print("  ALL 5 PREDICTIVE RISK ANALYZER TESTS PASSED! (5/5 PASS)")
    print("=" * 80)
