"""
Test Suite: Operator Confirmation & Authorization Safeguards (Step 10)
Verifies strict human-in-the-loop requirement, atomic route switching,
rollback safety, and operator rejection handling.
"""
import pytest
from mission.mission_rerouting_manager import (
    MissionReroutingManager,
    MissionReroutingConfig,
    ProposalStatus,
    RouteProposal
)
from mission.mission_state import GeographicPosition, Waypoint
from mission.navigation import SpatialNavigationEngine
from mission.decision_engine import MissionDecisionAction, MissionDecisionResult
from mission.trajectory_optimizer import CandidateRoute
from digital_twin.uav_digital_twin import (
    UAVDigitalTwinState,
    EnvironmentSubsystemState,
    EngineSubsystemState,
    BatterySubsystemState,
    ElectricMachineSubsystemState,
    HybridPowertrainSubsystemState,
    DigitalTwinQualityStatus
)


def _make_state_and_proposal(
    quality: DigitalTwinQualityStatus = DigitalTwinQualityStatus.HEALTHY,
    t: float = 100.0,
    feasible: bool = True
):
    nav = SpatialNavigationEngine()
    pos = GeographicPosition(18.6000, 73.9500, 3000.0, timestamp_s=t)
    nav.update(pos, ground_speed_mps=55.0, timestamp_s=t)

    state = UAVDigitalTwinState(
        timestamp_s=t,
        quality_status=quality,
        environment=EnvironmentSubsystemState(t, 9842.0, 3000.0, -4.5, 0.70, 0.90, 55.0, "CRUISE"),
        engine=EngineSubsystemState(t, 3200.0, 110.0, 82000.0, 72.0, 1.45, 4.5, 90.0, 85.0, 28.0, [750.0]*4, [175.0]*4, 1.2),
        battery=BatterySubsystemState(t, 85.0, 100.0, 350.0, 350.0, 0.08, 0.0, 0.0, 0.0, 0.0, 18000.0, 18.0, 25.0, "IDLE", False),
        electric_machine=ElectricMachineSubsystemState(t, "IDLE", 0.0, 0.0, 0.0, 3200.0, 0.94, 0.0, 30.0, False),
        powertrain=HybridPowertrainSubsystemState(t, "ENGINE_ONLY", "AUTO", 82000.0, 82000.0, 0.0, 82000.0, 0.0, "ENGINE_TO_PROP", False),
        mission=nav.update(pos, 55.0, t)
    )

    state.decision = MissionDecisionResult(
        recommended_action=MissionDecisionAction.ABORT_AND_RTB.value,
        decision_reason="Contingency triggered for testing",
        decision_confidence=1.0,
        triggered_conditions=["TEST_ALERT"],
        mission_feasible=False,
        rtb_feasible=True,
        alternate_feasible=True,
        selected_destination="Pune Lohegaon (Direct RTB)",
        decision_timestamp=t,
        data_quality_status=quality.value
    )

    cand = CandidateRoute(
        route_id="ROUTE_DIRECT_RTB",
        route_type="DIRECT_RTB",
        destination_name="Pune Lohegaon (Direct RTB)",
        destination_position=GeographicPosition(18.5822, 73.9197, 592.0),
        waypoint_names=["WP-RTB-1", "WP-RTB-2"],
        waypoints=[
            GeographicPosition(18.5822, 73.9197, 592.0),
            GeographicPosition(18.7000, 74.0000, 1500.0)
        ],
        total_distance_m=95000.0,
        total_duration_s=1727.0,
        total_fuel_liters=13.4,
        total_battery_energy_wh=2100.0,
        remaining_fuel_liters=36.6,
        remaining_battery_soc_pct=75.0,
        reserve_margin_pct=25.0,
        is_feasible=feasible,
        rejection_reasons=["Fuel reserve breach"] if not feasible else [],
        multi_criteria_cost=11.5,
        selection_rank=1
    )

    rm = MissionReroutingManager()
    rm.evaluate_and_update(state, [cand])
    prop = rm.get_pending_proposal()
    return rm, nav, state, prop


def test_approved_proposal_changes_simulated_route():
    rm, nav, state, prop = _make_state_and_proposal(t=100.0)
    assert prop is not None
    assert nav.active_route_id == "ORIGINAL_MISSION_ROUTE"

    success, msg, p_out = rm.approve_proposal(
        proposal_id=prop.proposal_id,
        operator_id="PILOT-GCS-01",
        navigation_engine=nav,
        twin_state=state,
        current_time_s=100.0
    )
    assert success is True
    assert p_out.status == ProposalStatus.APPLIED
    assert nav.active_route_id == "ROUTE_DIRECT_RTB"
    assert rm.active_route_id == "ROUTE_DIRECT_RTB"
    assert nav.route_version == 2


def test_rejected_proposal_leaves_active_route_unchanged():
    rm, nav, state, prop = _make_state_and_proposal(t=100.0)
    assert prop is not None

    success, msg, p_out = rm.reject_proposal(
        proposal_id=prop.proposal_id,
        reason="Operator decided to maintain altitude and continue",
        operator_id="PILOT-GCS-01",
        current_time_s=100.0
    )
    assert success is True
    assert p_out.status == ProposalStatus.REJECTED
    assert nav.active_route_id == "ORIGINAL_MISSION_ROUTE"
    assert rm.active_route_id == "ORIGINAL_MISSION_ROUTE"
    assert rm.get_pending_proposal() is None


def test_unknown_proposal_id_is_rejected():
    rm, nav, state, prop = _make_state_and_proposal(t=100.0)
    success, msg, p_out = rm.approve_proposal("NON_EXISTENT_PROPOSAL", operator_id="PILOT-GCS-01")
    assert success is False
    assert "Unknown proposal ID" in msg
    assert p_out is None


def test_expired_proposal_cannot_be_approved():
    rm, nav, state, prop = _make_state_and_proposal(t=100.0)
    assert prop is not None

    # Attempt approval at t=250.0s (past the 90s expiry duration)
    success, msg, p_out = rm.approve_proposal(
        proposal_id=prop.proposal_id,
        operator_id="PILOT-GCS-01",
        navigation_engine=nav,
        twin_state=state,
        current_time_s=250.0
    )
    assert success is False
    assert "expired" in msg.lower()
    assert p_out.status == ProposalStatus.EXPIRED
    assert nav.active_route_id == "ORIGINAL_MISSION_ROUTE"


def test_infeasible_proposal_cannot_be_approved():
    rm, nav, state, prop = _make_state_and_proposal(t=100.0, feasible=False)
    assert prop is not None
    assert prop.feasibility_status is False

    success, msg, p_out = rm.approve_proposal(
        proposal_id=prop.proposal_id,
        operator_id="PILOT-GCS-01",
        navigation_engine=nav,
        twin_state=state,
        current_time_s=100.0
    )
    assert success is False
    assert "infeasible" in msg.lower()
    assert nav.active_route_id == "ORIGINAL_MISSION_ROUTE"


def test_invalid_twin_quality_blocks_approval():
    rm, nav, state, prop = _make_state_and_proposal(t=100.0)
    assert prop is not None

    # Degrade digital twin state to INVALID
    state.quality_status = DigitalTwinQualityStatus.INVALID

    success, msg, p_out = rm.approve_proposal(
        proposal_id=prop.proposal_id,
        operator_id="PILOT-GCS-01",
        navigation_engine=nav,
        twin_state=state,
        current_time_s=100.0
    )
    assert success is False
    assert "Digital Twin data quality is INVALID" in msg
    assert nav.active_route_id == "ORIGINAL_MISSION_ROUTE"


def test_duplicate_approval_attempt_blocked():
    rm, nav, state, prop = _make_state_and_proposal(t=100.0)
    # First approval
    s1, _, _ = rm.approve_proposal(prop.proposal_id, "PILOT-GCS-01", nav, state, 100.0)
    assert s1 is True

    # Second approval attempt on already applied proposal
    s2, msg2, _ = rm.approve_proposal(prop.proposal_id, "PILOT-GCS-01", nav, state, 101.0)
    assert s2 is False
    assert "already been applied" in msg2


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
