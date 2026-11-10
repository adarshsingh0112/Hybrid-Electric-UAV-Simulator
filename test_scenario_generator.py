"""
DRDO MALE UAV Multi-Scenario Predictive Simulation (Step 9)
Root Test Runner Proxy: Scenario Generator & Uncertainty Distributions
"""
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from tests.test_scenario_generator import (
    test_1_nominal_scenario,
    test_2_bounded_best_and_worst_cases,
    test_3_fixed_seed_reproducibility,
    test_4_monte_carlo_distributions_and_clipping,
    test_5_full_ensemble_generation,
    test_6_configuration_validation_and_errors,
)

if __name__ == "__main__":
    print("=" * 80)
    print("  SCENARIO GENERATOR & MONTE CARLO TEST SUITE (STEP 9)")
    print("=" * 80)
    test_1_nominal_scenario()
    print()
    test_2_bounded_best_and_worst_cases()
    print()
    test_3_fixed_seed_reproducibility()
    print()
    test_4_monte_carlo_distributions_and_clipping()
    print()
    test_5_full_ensemble_generation()
    print()
    test_6_configuration_validation_and_errors()
    print()
    print("=" * 80)
    print("  ALL 6 SCENARIO GENERATOR TESTS PASSED! (6/6 PASS)")
    print("=" * 80)
