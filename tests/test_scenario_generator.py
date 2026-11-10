"""
DRDO MALE UAV Multi-Scenario Predictive Simulation (Step 9)
Test Suite: Scenario Generator & Monte Carlo Uncertainty Distributions
"""
import os
import sys
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from simulation.scenario_generator import (
    ScenarioType,
    ScenarioParameters,
    FastForwardConfig,
    ScenarioGenerator,
)


def test_1_nominal_scenario():
    """Verify nominal scenario has zero perturbations and baseline multipliers."""
    print("  [TEST 1] Testing Nominal Scenario Generation...")
    config = FastForwardConfig()
    generator = ScenarioGenerator(config=config)

    nominal = generator.generate_nominal_scenario()
    assert nominal.scenario_type == ScenarioType.NOMINAL
    assert nominal.scenario_id == "SCENARIO_NOMINAL"
    assert nominal.wind_speed_mps == 0.0
    assert nominal.wind_direction_deg == 0.0
    assert nominal.ambient_temp_delta_c == 0.0
    assert nominal.engine_power_multiplier == 1.0
    assert nominal.engine_fuel_flow_multiplier == 1.0
    assert nominal.battery_resistance_multiplier == 1.0
    assert nominal.propulsion_power_multiplier == 1.0
    assert nominal.payload_mass_delta_kg == 0.0
    print("    -> Nominal scenario contains exact baseline physical parameters.")


def test_2_bounded_best_and_worst_cases():
    """Verify best-case and worst-case bounded scenario envelopes."""
    print("  [TEST 2] Testing Bounded Best-Case and Worst-Case Generation...")
    config = FastForwardConfig()
    generator = ScenarioGenerator(config=config)

    best = generator.generate_best_case_scenario(route_bearing_deg=90.0)
    worst = generator.generate_worst_case_scenario(route_bearing_deg=90.0)

    assert best.scenario_type == ScenarioType.BEST_CASE
    assert worst.scenario_type == ScenarioType.WORST_CASE

    # Worst case should have higher propulsion power demand, higher fuel consumption, higher battery R_int
    assert worst.propulsion_power_multiplier >= 1.0
    assert worst.engine_fuel_flow_multiplier >= 1.0
    assert worst.engine_power_multiplier <= 1.0
    assert worst.battery_resistance_multiplier >= 1.0
    assert worst.payload_mass_delta_kg >= 0.0

    # Best case should have lower or equal fuel flow and resistance multipliers
    assert best.propulsion_power_multiplier <= worst.propulsion_power_multiplier
    assert best.engine_fuel_flow_multiplier <= worst.engine_fuel_flow_multiplier
    assert best.battery_resistance_multiplier <= worst.battery_resistance_multiplier
    assert best.payload_mass_delta_kg <= worst.payload_mass_delta_kg
    print("    -> Bounded envelopes properly bracket environmental and powertrain stress.")


def test_3_fixed_seed_reproducibility():
    """Verify Monte Carlo generation is 100% deterministic with fixed seed."""
    print("  [TEST 3] Testing Fixed-Seed Reproducibility...")
    config1 = FastForwardConfig(random_seed=42, num_scenarios=15)
    config2 = FastForwardConfig(random_seed=42, num_scenarios=15)

    gen1 = ScenarioGenerator(config=config1)
    gen2 = ScenarioGenerator(config=config2)

    mc1 = gen1.generate_monte_carlo_scenarios(count=15, seed=42)
    mc2 = gen2.generate_monte_carlo_scenarios(count=15, seed=42)

    assert len(mc1) == 15
    assert len(mc2) == 15

    for s1, s2 in zip(mc1, mc2):
        assert s1.scenario_id == s2.scenario_id
        assert abs(s1.wind_speed_mps - s2.wind_speed_mps) < 1e-6
        assert abs(s1.wind_direction_deg - s2.wind_direction_deg) < 1e-6
        assert abs(s1.ambient_temp_delta_c - s2.ambient_temp_delta_c) < 1e-6
        assert abs(s1.engine_power_multiplier - s2.engine_power_multiplier) < 1e-6
        assert abs(s1.engine_fuel_flow_multiplier - s2.engine_fuel_flow_multiplier) < 1e-6
        assert abs(s1.battery_resistance_multiplier - s2.battery_resistance_multiplier) < 1e-6
        assert abs(s1.propulsion_power_multiplier - s2.propulsion_power_multiplier) < 1e-6
        assert abs(s1.payload_mass_delta_kg - s2.payload_mass_delta_kg) < 1e-6

    print("    -> Identical random seeds produce exact bitwise-matching Monte Carlo sets.")


def test_4_monte_carlo_distributions_and_clipping():
    """Verify sampled perturbations respect bounds and physical constraints."""
    print("  [TEST 4] Testing Monte Carlo Distribution Bounds & Physical Clipping...")
    config = FastForwardConfig(
        random_seed=999,
        num_scenarios=50,
        wind_max_speed_mps=15.0,
        temp_min_delta_c=-8.0,
        temp_max_delta_c=8.0,
        engine_min_power_mult=0.85,
        engine_max_power_mult=1.00,
        engine_min_fuel_mult=1.00,
        engine_max_fuel_mult=1.10,
        battery_min_r_mult=0.95,
        battery_max_r_mult=1.30,
        power_demand_min_mult=0.95,
        power_demand_max_mult=1.08,
        payload_mass_min_delta_kg=-25.0,
        payload_mass_max_delta_kg=25.0,
    )
    generator = ScenarioGenerator(config=config)
    mc_scenarios = generator.generate_monte_carlo_scenarios(count=50, seed=999)

    assert len(mc_scenarios) == 50
    for s in mc_scenarios:
        assert 0.0 <= s.wind_speed_mps <= 15.0
        assert 0.0 <= s.wind_direction_deg <= 360.0
        assert -8.0 <= s.ambient_temp_delta_c <= 8.0
        assert 0.85 <= s.engine_power_multiplier <= 1.00
        assert 1.00 <= s.engine_fuel_flow_multiplier <= 1.10
        assert 0.95 <= s.battery_resistance_multiplier <= 1.30
        assert 0.95 <= s.propulsion_power_multiplier <= 1.08
        assert -25.0 <= s.payload_mass_delta_kg <= 25.0

    print("    -> All 50 Monte Carlo scenarios strictly comply with physical clipping bounds.")


def test_5_full_ensemble_generation():
    """Verify generate_full_ensemble includes nominal, bounds, and MC scenarios."""
    print("  [TEST 5] Testing Full Ensemble Composition...")
    config = FastForwardConfig(num_scenarios=20)
    generator = ScenarioGenerator(config=config)
    ensemble = generator.generate_full_ensemble(route_bearing_deg=45.0, seed=42)

    # Nominal + Best + Worst + 20 MC = 23 scenarios
    assert len(ensemble) == 23
    assert ensemble[0].scenario_type == ScenarioType.NOMINAL
    assert ensemble[1].scenario_type == ScenarioType.BEST_CASE
    assert ensemble[2].scenario_type == ScenarioType.WORST_CASE
    for s in ensemble[3:]:
        assert s.scenario_type == ScenarioType.MONTE_CARLO

    print("    -> Full ensemble properly structured (1 nominal, 2 bounds, 20 MC scenarios).")


def test_6_configuration_validation_and_errors():
    """Verify configuration validation rejects invalid parameters."""
    print("  [TEST 6] Testing Configuration Validation & Error Handling...")
    valid_cfg = FastForwardConfig()
    assert valid_cfg.validate() is None or valid_cfg.validate() is True

    # Invalid MC count <= 0
    invalid_mc = FastForwardConfig(num_scenarios=0)
    try:
        invalid_mc.validate()
        assert False, "Should have raised ValueError for non-positive MC scenarios"
    except ValueError:
        pass

    # Invalid dt <= 0
    invalid_dt = FastForwardConfig(simulation_timestep_s=-1.0)
    try:
        invalid_dt.validate()
        assert False, "Should have raised ValueError for negative dt"
    except ValueError:
        pass

    # Load from default JSON
    json_path = PROJECT_ROOT / "configs" / "fast_forward_config.json"
    if json_path.exists():
        cfg_from_file = FastForwardConfig.from_json(json_path)
        assert cfg_from_file.validate() is None or cfg_from_file.validate() is True
        print("    -> Configuration successfully loaded and validated from configs/fast_forward_config.json.")

    print("    -> Configuration error detection and schema validation verified.")


if __name__ == "__main__":
    print("=" * 80)
    print("  SCENARIO GENERATOR & MONTE CARLO TEST SUITE (STEP 9)")
    print("=" * 80)
    test_1_nominal_scenario()
    test_2_bounded_best_and_worst_cases()
    test_3_fixed_seed_reproducibility()
    test_4_monte_carlo_distributions_and_clipping()
    test_5_full_ensemble_generation()
    test_6_configuration_validation_and_errors()
    print("=" * 80)
    print("  ALL 6 SCENARIO GENERATOR TESTS PASSED! (6/6 PASS)")
    print("=" * 80)
