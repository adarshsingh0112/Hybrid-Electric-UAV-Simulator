"""
DRDO MALE UAV AI-Guided Mission Decision Engine Test Suite (Step 8)
Verifies deterministic multi-subsystem decision making, safety reserve policy enforcement,
confidence calibration, explainable condition triggers, and explicit missing-data handling.

Test Scenarios:
1. Nominal Healthy Mission Sortie -> CONTINUE_MISSION (confidence >= 0.95, mission_feasible=True).
2. Low Battery Reserve Violation (<20% SoC) -> ABORT_AND_RTB to protect reserve buffer.
3. Critically Depleted Battery (<8% SoC) with Engine Impairment -> EMERGENCY_DESCENT.
4. Low Fuel Reserve Violation (<18 L) -> ABORT_AND_RTB while fuel remains.
5. Degraded Engine Mechanical Health (<65%) -> ABORT_AND_RTB / DIVERT_TO_ALTERNATE.
6. Insufficient Return Energy for Home Base -> DIVERT_TO_ALTERNATE (recovers at reachable airfield).
7. Critical Dual Propulsion Loss -> EMERGENCY_DESCENT.
8. Critical Battery Overheat (>65°C) -> EMERGENCY_DESCENT to prevent thermal runaway.
9. Invalid Corrupted Telemetry (NaN / Impossible Bounds) -> Clamped confidence, ABORT_AND_RTB.
10. Stale Subsystem Telemetry Lag (>200ms) -> DEGRADED data quality, confidence penalty, ABORT_AND_RTB.
11. Explicit Missing Critical Inputs -> Graceful handling, confidence <= 0.10, ABORT_AND_RTB.
12. Safety Configuration Schema Validation -> Rejects non-normalizing weights and negative scales.
"""

import sys
import math
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent if (Path(__file__).resolve().parent.name == "tests") else Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from mission.decision_engine import (
    MissionDecisionEngine,
    MissionDecisionAction,
    MissionDecisionResult,
    DecisionEngineConfig,
    DestinationInfo,
)
from mission.reliability_engine import (
    MissionReliabilityEngine,
    MissionReliabilityState,
    ReliabilityScoreBreakdown,
    RTBPredictionResult,
    ReliabilityStatusLevel,
    ContingencyAction,
)
from mission.mission_state import GeographicPosition


def create_mock_reliability_state(
    mri: float = 95.0,
    status: str = "NOMINAL",
    action: str = "CONTINUE_MISSION",
    rtb_feasible: bool = True,
    margin_pct: float = 100.0,
    alt_feasible: bool = True,
    alt_id: str = "ALT-01: Baramati"
) -> MissionReliabilityState:
    breakdown = ReliabilityScoreBreakdown(
        energy_reserve_factor=95.0,
        propulsion_health_factor=98.0,
        power_headroom_factor=95.0,
        telemetry_quality_factor=100.0,
        mission_progress_factor=100.0,
        composite_score=mri
    )
    rtb = RTBPredictionResult(
        distance_to_home_m=45000.0,
        estimated_duration_s=818.0,
        estimated_power_w=52000.0,
        estimated_energy_wh=12000.0,
        estimated_fuel_liters=6.2,
        available_battery_wh=18000.0,
        available_usable_battery_wh=14400.0,
        available_fuel_liters=140.0,
        available_usable_fuel_liters=122.0,
        available_total_energy_wh=468000.0,
        energy_margin_pct=margin_pct,
        rtb_feasible=rtb_feasible,
        limiting_factor="NONE_FEASIBLE" if rtb_feasible else "INSUFFICIENT_FUEL",
        nearest_alternate_id=alt_id,
        nearest_alternate_distance_m=42000.0,
        nearest_alternate_feasible=alt_feasible
    )
    return MissionReliabilityState(
        timestamp_s=100.0,
        mission_reliability_index=mri,
        reliability_status=status,
        rtb_feasible=rtb_feasible,
        estimated_rtb_time_s=818.0,
        estimated_rtb_energy_wh=12000.0,
        estimated_rtb_fuel_liters=6.2,
        available_fuel_liters=140.0,
        available_battery_wh=18000.0,
        contingency_recommendation=action,
        contingency_reason="Nominal sortie.",
        available_energy_margin_pct=margin_pct,
        score_breakdown=breakdown,
        rtb_prediction=rtb,
        provenance="SIMULATED"
    )


def test_1_normal_healthy_sortie():
    print("[TEST 1] Testing Normal Healthy Mission Sortie...")
    engine = MissionDecisionEngine()
    rel = create_mock_reliability_state()

    decision = engine.evaluate(
        timestamp_s=100.0,
        quality_status="HEALTHY",
        engine_health_pct=98.5,
        battery_soc_pct=95.0,
        battery_usable_kwh=17.1,
        battery_temp_c=28.0,
        battery_is_constrained=False,
        remaining_fuel_liters=140.0,
        power_deficit_w=0.0,
        propulsion_power_req_w=52000.0,
        distance_to_home_m=45000.0,
        remaining_route_distance_m=120000.0,
        reliability_state=rel
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Decision Confidence: {decision.decision_confidence:.3f}")
    print(f"  -> Reason: {decision.decision_reason}")
    print(f"  -> Mission Feasible: {decision.mission_feasible} | RTB Feasible: {decision.rtb_feasible}")

    assert decision.recommended_action == MissionDecisionAction.CONTINUE_MISSION.value
    assert decision.decision_confidence >= 0.95
    assert decision.mission_feasible is True
    assert decision.rtb_feasible is True
    assert "ALL_PRECONDITIONS_NOMINAL" in decision.triggered_conditions
    print("  -> PASSED: Nominal conditions produce CONTINUE_MISSION with high confidence.")


def test_2_low_battery_reserve_violation():
    print("[TEST 2] Testing Low Battery Reserve Violation (<20% Policy Limit)...")
    engine = MissionDecisionEngine()
    rel = create_mock_reliability_state(mri=70.0, status="CAUTION", action="ABORT_AND_RTB")

    decision = engine.evaluate(
        timestamp_s=105.0,
        quality_status="HEALTHY",
        engine_health_pct=98.0,
        battery_soc_pct=15.0,   # Violates 20% limit
        battery_usable_kwh=2.7,
        battery_temp_c=32.0,
        battery_is_constrained=False,
        remaining_fuel_liters=120.0,
        power_deficit_w=0.0,
        propulsion_power_req_w=52000.0,
        distance_to_home_m=45000.0,
        remaining_route_distance_m=120000.0,
        reliability_state=rel
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Reason: {decision.decision_reason}")
    print(f"  -> Triggered Conditions: {decision.triggered_conditions}")

    assert decision.recommended_action == MissionDecisionAction.ABORT_AND_RTB.value
    assert decision.mission_feasible is False
    assert decision.rtb_feasible is True
    assert "BATTERY_SAFETY_RESERVE_BREACH" in decision.triggered_conditions
    print("  -> PASSED: Battery reserve violation safely mandates ABORT_AND_RTB.")


def test_3_critically_depleted_battery():
    print("[TEST 3] Testing Critically Depleted Battery (<8% SoC) with Engine Degradation...")
    engine = MissionDecisionEngine()
    rel = create_mock_reliability_state(mri=30.0, status="CRITICAL", action="EMERGENCY_DESCENT")

    decision = engine.evaluate(
        timestamp_s=110.0,
        quality_status="HEALTHY",
        engine_health_pct=35.0,  # Below 40% critical threshold
        battery_soc_pct=6.5,    # Below 8% critical reserve limit
        battery_usable_kwh=1.1,
        battery_temp_c=40.0,
        battery_is_constrained=True,
        remaining_fuel_liters=25.0,
        power_deficit_w=15000.0,
        propulsion_power_req_w=75000.0,
        distance_to_home_m=80000.0,
        remaining_route_distance_m=150000.0,
        reliability_state=rel
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Reason: {decision.decision_reason}")

    assert decision.recommended_action == MissionDecisionAction.EMERGENCY_DESCENT.value
    assert decision.mission_feasible is False
    assert decision.rtb_feasible is False
    print("  -> PASSED: Critically depleted propulsion reserves trigger EMERGENCY_DESCENT.")


def test_4_low_fuel_reserve_violation():
    print("[TEST 4] Testing Low Fuel Reserve Violation (<18 L Policy Limit)...")
    engine = MissionDecisionEngine()
    rel = create_mock_reliability_state(mri=68.0, status="CAUTION", action="ABORT_AND_RTB")

    decision = engine.evaluate(
        timestamp_s=115.0,
        quality_status="HEALTHY",
        engine_health_pct=95.0,
        battery_soc_pct=85.0,
        battery_usable_kwh=15.3,
        battery_temp_c=30.0,
        battery_is_constrained=False,
        remaining_fuel_liters=14.5,  # Violates 18.0 L reserve limit
        power_deficit_w=0.0,
        propulsion_power_req_w=52000.0,
        distance_to_home_m=35000.0,
        remaining_route_distance_m=90000.0,
        reliability_state=rel
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Reason: {decision.decision_reason}")

    assert decision.recommended_action == MissionDecisionAction.ABORT_AND_RTB.value
    assert decision.mission_feasible is False
    assert "FUEL_SAFETY_RESERVE_BREACH" in decision.triggered_conditions
    print("  -> PASSED: Fuel reserve breach triggers ABORT_AND_RTB to preserve reserve buffer.")


def test_5_degraded_engine_mechanical_health():
    print("[TEST 5] Testing Degraded Engine Mechanical Health (<65%)...")
    engine = MissionDecisionEngine()
    rel = create_mock_reliability_state(mri=55.0, status="CAUTION", action="ABORT_AND_RTB")

    decision = engine.evaluate(
        timestamp_s=120.0,
        quality_status="HEALTHY",
        engine_health_pct=52.0,   # Degraded between 40% and 65%
        battery_soc_pct=90.0,
        battery_usable_kwh=16.2,
        battery_temp_c=32.0,
        battery_is_constrained=False,
        remaining_fuel_liters=90.0,
        power_deficit_w=0.0,
        propulsion_power_req_w=50000.0,
        distance_to_home_m=40000.0,
        remaining_route_distance_m=100000.0,
        reliability_state=rel
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Reason: {decision.decision_reason}")

    assert decision.recommended_action in [
        MissionDecisionAction.ABORT_AND_RTB.value,
        MissionDecisionAction.DIVERT_TO_ALTERNATE.value
    ]
    assert decision.mission_feasible is False
    assert "ENGINE_HEALTH_DEGRADED_ABORT" in decision.triggered_conditions
    print("  -> PASSED: Degraded mechanical engine health safely aborts mission.")


def test_6_insufficient_return_energy_to_home():
    print("[TEST 6] Testing Insufficient Return Energy for Home Base (Divert to Alternate)...")
    engine = MissionDecisionEngine()
    # RTB is infeasible, but alternate recovery is feasible
    rel = create_mock_reliability_state(
        mri=48.0,
        status="CRITICAL",
        action="DIVERT_TO_ALTERNATE",
        rtb_feasible=False,
        margin_pct=-25.0,
        alt_feasible=True,
        alt_id="ALT-01: Baramati"
    )

    decision = engine.evaluate(
        timestamp_s=125.0,
        quality_status="HEALTHY",
        engine_health_pct=88.0,
        battery_soc_pct=40.0,
        battery_usable_kwh=7.2,
        battery_temp_c=35.0,
        battery_is_constrained=False,
        remaining_fuel_liters=22.0,
        power_deficit_w=0.0,
        propulsion_power_req_w=50000.0,
        distance_to_home_m=280000.0,   # Far from home base (280 km)
        remaining_route_distance_m=350000.0,
        reliability_state=rel
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Selected Destination: {decision.selected_destination}")
    print(f"  -> Reason: {decision.decision_reason}")

    assert decision.recommended_action == MissionDecisionAction.DIVERT_TO_ALTERNATE.value
    assert decision.rtb_feasible is False
    assert decision.alternate_feasible is True
    assert "HOME_BASE_RTB_INFEASIBLE" in decision.triggered_conditions
    print("  -> PASSED: Infeasible RTB routes UAV to nearest reachable alternate recovery base.")


def test_7_dual_propulsion_loss_emergency():
    print("[TEST 7] Testing Dual Propulsion Loss Emergency...")
    engine = MissionDecisionEngine()
    rel = create_mock_reliability_state(mri=15.0, status="CRITICAL", action="EMERGENCY_DESCENT", rtb_feasible=False, alt_feasible=False)

    decision = engine.evaluate(
        timestamp_s=130.0,
        quality_status="HEALTHY",
        engine_health_pct=15.0,   # Catastrophic engine failure (<40%)
        battery_soc_pct=10.0,    # Depleted battery
        battery_usable_kwh=1.8,
        battery_temp_c=45.0,
        battery_is_constrained=True,
        remaining_fuel_liters=5.0,
        power_deficit_w=35000.0,  # Massive deficit
        propulsion_power_req_w=60000.0,
        distance_to_home_m=120000.0,
        remaining_route_distance_m=160000.0,
        reliability_state=rel
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Reason: {decision.decision_reason}")

    assert decision.recommended_action == MissionDecisionAction.EMERGENCY_DESCENT.value
    assert decision.mission_feasible is False
    assert decision.rtb_feasible is False
    assert decision.alternate_feasible is False
    print("  -> PASSED: Total propulsion loss mandates immediate EMERGENCY_DESCENT.")


def test_8_critical_battery_overheat():
    print("[TEST 8] Testing Critical Battery Thermal Overheat (>65°C)...")
    engine = MissionDecisionEngine()
    rel = create_mock_reliability_state(mri=35.0, status="CRITICAL", action="EMERGENCY_DESCENT")

    decision = engine.evaluate(
        timestamp_s=135.0,
        quality_status="HEALTHY",
        engine_health_pct=98.0,
        battery_soc_pct=75.0,
        battery_usable_kwh=13.5,
        battery_temp_c=68.5,     # Critical thermal event (> 65.0 C)
        battery_is_constrained=True,
        remaining_fuel_liters=100.0,
        power_deficit_w=0.0,
        propulsion_power_req_w=50000.0,
        distance_to_home_m=45000.0,
        remaining_route_distance_m=120000.0,
        reliability_state=rel
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Reason: {decision.decision_reason}")

    assert decision.recommended_action == MissionDecisionAction.EMERGENCY_DESCENT.value
    assert "BATTERY_THERMAL_CRITICAL" in decision.triggered_conditions
    print("  -> PASSED: Dangerous battery thermal state triggers EMERGENCY_DESCENT.")


def test_9_invalid_corrupted_telemetry():
    print("[TEST 9] Testing Corrupted / Invalid Telemetry Injection...")
    engine = MissionDecisionEngine()

    decision = engine.evaluate(
        timestamp_s=140.0,
        quality_status="INVALID",
        engine_health_pct=98.0,
        battery_soc_pct=float("nan"),  # NaN telemetry
        battery_usable_kwh=16.0,
        battery_temp_c=30.0,
        battery_is_constrained=False,
        remaining_fuel_liters=110.0,
        power_deficit_w=0.0,
        propulsion_power_req_w=50000.0,
        distance_to_home_m=40000.0,
        remaining_route_distance_m=100000.0
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Confidence: {decision.decision_confidence:.2f}")
    print(f"  -> Data Quality: {decision.data_quality_status}")

    assert decision.recommended_action == MissionDecisionAction.ABORT_AND_RTB.value
    assert decision.decision_confidence <= 0.40
    assert decision.mission_feasible is False
    assert "TELEMETRY_INVALID_CORRUPTION" in decision.triggered_conditions
    print("  -> PASSED: Telemetry corruption penalizes confidence and enforces abort.")


def test_10_stale_telemetry_lag():
    print("[TEST 10] Testing Stale Telemetry Lag (>200ms)...")
    engine = MissionDecisionEngine()

    decision = engine.evaluate(
        timestamp_s=145.0,
        quality_status="DEGRADED",
        engine_health_pct=95.0,
        battery_soc_pct=80.0,
        battery_usable_kwh=14.4,
        battery_temp_c=32.0,
        battery_is_constrained=False,
        remaining_fuel_liters=100.0,
        power_deficit_w=0.0,
        propulsion_power_req_w=50000.0,
        distance_to_home_m=40000.0,
        remaining_route_distance_m=100000.0,
        stale_data_flags={"mission": True, "engine": False, "battery": False}
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Confidence: {decision.decision_confidence:.2f}")

    assert decision.recommended_action == MissionDecisionAction.ABORT_AND_RTB.value
    assert decision.decision_confidence <= 0.75
    assert "SUBSYSTEM_TELEMETRY_STALE" in decision.triggered_conditions
    print("  -> PASSED: Stale telemetry lag triggers abort and degrades confidence.")


def test_11_explicit_missing_critical_telemetry():
    print("[TEST 11] Testing Explicit Missing Critical Inputs...")
    engine = MissionDecisionEngine()

    # Pass None for essential fields
    decision = engine.evaluate(
        timestamp_s=None,
        quality_status=None,
        engine_health_pct=None,
        battery_soc_pct=None,
        battery_usable_kwh=None,
        battery_temp_c=None,
        battery_is_constrained=None,
        remaining_fuel_liters=None,
        power_deficit_w=None,
        propulsion_power_req_w=None,
        distance_to_home_m=None,
        remaining_route_distance_m=None
    )

    print(f"  -> Recommended Action: {decision.recommended_action}")
    print(f"  -> Confidence: {decision.decision_confidence:.2f}")
    print(f"  -> Triggered Conditions: {decision.triggered_conditions}")

    assert decision.recommended_action == MissionDecisionAction.ABORT_AND_RTB.value
    assert decision.decision_confidence <= 0.10
    assert "MISSING_CRITICAL_TELEMETRY" in decision.triggered_conditions
    print("  -> PASSED: Missing inputs rejected safely without crashing or defaulting to zero.")


def test_12_config_validation_and_safety_checks():
    print("[TEST 12] Testing Safety Configuration Schema Validation...")
    # 1. Valid config passes
    cfg_valid = DecisionEngineConfig()
    cfg_valid.validate()

    # 2. Invalid weights sum != 1.0
    bad_weights = {"weight_fuel": 0.50, "weight_battery": 0.20} # sum = 0.70 != 1.0
    cfg_bad_w = DecisionEngineConfig(route_optimization_weights=bad_weights)
    try:
        cfg_bad_w.validate()
        assert False, "Should have raised ValueError on weights summing to 0.70"
    except ValueError as e:
        print(f"  -> Caught invalid weight sum: {e}")

    # 3. Negative normalization scale
    bad_scales = {"norm_distance_m": -100.0, "norm_duration_s": 5400.0, "norm_fuel_l": 35.0, "norm_battery_wh": 18000.0}
    cfg_bad_s = DecisionEngineConfig(normalization_scales=bad_scales)
    try:
        cfg_bad_s.validate()
        assert False, "Should have raised ValueError on negative scale"
    except ValueError as e:
        print(f"  -> Caught negative normalization scale: {e}")

    # 4. Inverted thermal thresholds
    cfg_bad_t = DecisionEngineConfig(critical_battery_temp_c=50.0, caution_battery_temp_c=60.0)
    try:
        cfg_bad_t.validate()
        assert False, "Should have raised ValueError on inverted thermal thresholds"
    except ValueError as e:
        print(f"  -> Caught inverted thermal thresholds: {e}")

    print("  -> PASSED: Strict configuration schema verification enforces mathematical consistency.")


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
