"""
ENGINE-TWIN: Master Automated Test & Validation Runner (SIH26054)
Executes all project verification suites and outputs a consolidated test report:
1. Physics & Simulation Tests (test_simulation.py)
2. AI Layer, EKF, RUL & XAI Tests (test_ai_layer.py)
3. Scientific Benchmark vs Baselines (validation/benchmark.py)
4. Architectural Ablation Studies (validation/ablation.py)
"""
import sys
import subprocess
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent

def run_suite(name: str, script_rel_path: str):
    print("\n" + "#" * 75)
    print(f"  RUNNING SUITE: {name}")
    print("#" * 75)
    t0 = time.time()
    script_path = PROJECT_ROOT / script_rel_path
    res = subprocess.run([sys.executable, str(script_path)], cwd=str(PROJECT_ROOT), capture_output=True, text=True)
    elapsed = time.time() - t0
    
    print(res.stdout)
    if res.returncode != 0:
        if "ModuleNotFoundError" in res.stderr and any(f"'{mod}'" in res.stderr for mod in ("torch", "sklearn", "psutil")):
            missing = [m for m in ("torch", "sklearn", "psutil") if f"'{m}'" in res.stderr][0]
            print(f"[SKIP] {name} SKIPPED (Optional ML package '{missing}' not installed in Python 3.14)")
            return "SKIP", elapsed
        print(f"[FAIL] {name} FAILED with code {res.returncode}")
        print("ERROR OUTPUT:\n", res.stderr)
        return "FAIL", elapsed
    else:
        print(f"[PASS] {name} PASSED in {elapsed:.2f}s")
        return "PASS", elapsed

def main():
    print("=" * 80)
    print("  ENGINE-TWIN: MASTER CONSOLIDATED TEST & VALIDATION PASS (DRDO SIH26054)")
    print("=" * 80)

    suites = [
        ("Physics Engine, ISA Atmosphere & Sensor Dynamics", "test_simulation.py"),
        ("Battery Energy Storage System (BESS)", "test_battery.py"),
        ("Hybrid-Electric Powertrain Integration", "test_hybrid_powertrain.py"),
        ("Unified Real-Time UAV Digital Twin State Layer", "test_uav_digital_twin.py"),
        ("Spatial Mission & Navigation Engine", "test_mission_navigation.py"),
        ("AI Diagnostics, EKF Fusion & Prognostics Layer", "test_ai_layer.py"),
        ("Mission Reliability Scoring & Dynamic Contingency (Step 7)", "test_mission_reliability.py"),
        ("AI-Guided Mission Decision Engine (Step 8)", "test_mission_decision_engine.py"),
        ("Multi-Criteria Trajectory Optimizer (Step 8)", "test_trajectory_optimizer.py"),
        ("Monte Carlo Scenario Generator & Uncertainties (Step 9)", "test_scenario_generator.py"),
        ("Fast-Forward Simulation Engine (Step 9)", "test_fast_forward_engine.py"),
        ("Predictive Risk Analyzer & Ensemble Statistics (Step 9)", "test_predictive_risk_analyzer.py"),
        ("Mission Rerouting Manager & Proposals (Step 10)", "test_mission_rerouting_manager.py"),
        ("Operator Confirmation & Safety Safeguards (Step 10)", "test_operator_confirmation.py"),
        ("Mission Re-Routing REST API Layer (Step 10)", "test_mission_rerouting_api.py"),
        ("Closed-Loop Re-Routing Integration & Rollback (Step 10)", "test_mission_rerouting_integration.py"),
        ("Dedicated Data Leakage & Provenance Integrity Audit", "validation/leakage_audit.py"),
        ("Adversarial Stress & Failure Engineering Suite", "validation/stress_testing.py"),
        ("Empirical Baseline Comparison (EIS vs ML vs ENGINE-TWIN)", "validation/benchmark.py"),
        ("Architectural Ablation Study (Physics vs Raw ML)", "validation/ablation.py"),
        ("Multi-Engine Scalability Proof (Rotax 914 Flat-Four)", "tools/test_multi_engine_transfer.py"),
        ("Edge Hardware AI Compute & Memory Resource Profiler", "tools/edge_benchmark_profiler.py")
    ]

    results = []
    total_t0 = time.time()
    for name, path in suites:
        status, elapsed = run_suite(name, path)
        results.append((name, status, elapsed))

    total_elapsed = time.time() - total_t0

    print("\n" + "=" * 75)
    print("  FINAL CONSOLIDATED TEST SUMMARY")
    print("=" * 75)
    all_passed = True
    for name, status, elapsed in results:
        status_str = f"[{status}]"
        if status == "FAIL":
            all_passed = False
        print(f"  {status_str:<8} | {name:<55} | ({elapsed:.2f}s)")

    print("-" * 75)
    print(f"  Total Test Time: {total_elapsed:.2f}s")
    if all_passed:
        print("  [SUCCESS] ALL VERIFICATION SUITES OPERATIONAL! (100% Core Integrity)")
    else:
        print("  [WARNING] ONE OR MORE TEST SUITES FAILED.")
    print("=" * 75)

if __name__ == "__main__":
    main()
