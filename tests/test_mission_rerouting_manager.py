"""
Unit Tests for Mission Rerouting Manager (Step 10)
Tests contingency-to-proposal creation, duplicate suppression, expiration,
and immutability safeguards.
"""
import pytest
from mission.mission_rerouting_manager import (
    MissionReroutingManager,
    MissionReroutingConfig,
    ProposalStatus,
    RouteProposal,
    AuditEvent
)
from mission.mission_state import GeographicPosition, Waypoint, MissionNavigationState
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


def _create_mock_twin_state(
    timestamp_s: float = 100.0,
    decision_action: MissionDecisionAction = MissionDecisionAction.CONTINUE_MISSION,
    quality_status: DigitalTwinQualityStatus = DigitalTwinQualityStatus.HEALTHY
) -> UAVDigitalTwinState:
    nav = SpatialNavigationEngine()
    pos = GeographicPosition(18.5822, 73.9197, 3000.0, timestamp_s=timestamp_s)
    nav_state = nav.update(pos, ground_speed_mps=55.0, timestamp_s=timestamp_s)

    env = EnvironmentSubsystemState(
        timestamp_s=timestamp_s, altitude_ft=9842.0, altitude_m=3000.0,
        ambient_temp_c=-4.5, ambient_pressure_bar=0.70, air_density_kgpm3=0.90,
        airspeed_mps=55.0, flight_phase="CRUISE"
    )
    eng = EngineSubsystemState(
        timestamp_s=timestamp_s, rpm=3200.0, power_hp=110.0, power_w=82000.0,
        throttle_pct=72.0, manifold_pressure_bar=1.45, oil_pressure_bar=4.5,
        oil_temp_c=90.0, coolant_temp_c=85.0, fuel_flow_lph=28.0,
        egt_c=[750.0]*4, cht_c=[175.0]*4, vibration_rms_g=1.2
    )
    batt = BatterySubsystemState(
        timestamp_s=timestamp_s, soc_pct=85.0, soh_pct=100.0,
        terminal_voltage_v=350.0, open_circuit_voltage_v=350.0, internal_resistance_ohm=0.08,
        current_a=0.0, power_demand_w=0.0, power_actual_w=0.0, power_loss_w=0.0,
        usable_energy_wh=18000.0, usable_energy_kwh=18.0, temperature_c=25.0,
        charge_state="IDLE", is_power_constrained=False
    )
    em = ElectricMachineSubsystemState(
        timestamp_s=timestamp_s, operating_mode="IDLE", mechanical_power_w=0.0,
        electrical_power_w=0.0, torque_nm=0.0, rpm=3200.0, efficiency=0.94,
        power_loss_w=0.0, temperature_c=30.0, is_thermal_warning=False
    )
    pt = HybridPowertrainSubsystemState(
        timestamp_s=timestamp_s, operating_mode="ENGINE_ONLY", control_mode="AUTO",
        propulsion_power_req_w=82000.0, propulsion_power_actual_w=82000.0,
        propulsion_power_deficit_w=0.0, engine_power_contribution_w=82000.0,
        electric_power_contribution_w=0.0, energy_flow_direction="ENGINE_TO_PROP",
        is_system_power_constrained=False
    )

    state = UAVDigitalTwinState(
        timestamp_s=timestamp_s,
        quality_status=quality_status,
        environment=env,
        engine=eng,
        battery=batt,
        electric_machine=em,
        powertrain=pt,
        mission=nav_state
    )

    state.decision = MissionDecisionResult(
        recommended_action=decision_action.value,
        decision_reason="Simulated contingency test trigger" if decision_action != MissionDecisionAction.CONTINUE_MISSION else "Nominal",
        decision_confidence=1.0,
        triggered_conditions=["TEST_RULE"] if decision_action != MissionDecisionAction.CONTINUE_MISSION else [],
        mission_feasible=(decision_action == MissionDecisionAction.CONTINUE_MISSION),
        rtb_feasible=True,
        alternate_feasible=True,
        selected_destination="Pune Lohegaon (Direct RTB)" if decision_action != MissionDecisionAction.CONTINUE_MISSION else "Pune Lohegaon",
        decision_timestamp=timestamp_s,
        data_quality_status=quality_status.value
    )
    return state


def _create_mock_candidate_routes() -> list:
    cand_rtb = CandidateRoute(
        route_id="ROUTE_DIRECT_RTB",
        route_type="DIRECT_RTB",
        destination_name="Pune Lohegaon (Direct RTB)",
        destination_position=GeographicPosition(18.5822, 73.9197, 592.0),
        waypoint_names=["WP-RTB-1", "WP-RTB-2", "WP-RTB-3"],
        waypoints=[
            GeographicPosition(18.5822, 73.9197, 592.0),
            GeographicPosition(18.7000, 74.0000, 1500.0),
            GeographicPosition(18.9000, 74.2000, 600.0)
        ],
        total_distance_m=95000.0,
        total_duration_s=1727.0,
        total_fuel_liters=13.4,
        total_battery_energy_wh=2100.0,
        remaining_fuel_liters=36.6,
        remaining_battery_soc_pct=75.0,
        reserve_margin_pct=25.0,
        is_feasible=True,
        multi_criteria_cost=11.5,
        selection_rank=1
    )
    return [cand_rtb]


def test_nominal_continuation_creates_no_proposal():
    rm = MissionReroutingManager()
    state = _create_mock_twin_state(timestamp_s=10.0, decision_action=MissionDecisionAction.CONTINUE_MISSION)
    cands = _create_mock_candidate_routes()

    telem = rm.evaluate_and_update(state, cands)
    assert telem.contingency_detected is False
    assert telem.pending_proposal_count == 0
    assert telem.active_proposal_id is None
    assert telem.operator_attention_required is False
    assert rm.get_pending_proposal() is None


def test_contingency_triggers_proposal_creation():
    rm = MissionReroutingManager()
    state = _create_mock_twin_state(timestamp_s=20.0, decision_action=MissionDecisionAction.ABORT_AND_RTB)
    cands = _create_mock_candidate_routes()

    telem = rm.evaluate_and_update(state, cands)
    assert telem.contingency_detected is True
    assert telem.pending_proposal_count == 1
    assert telem.active_proposal_id is not None
    assert telem.operator_attention_required is True

    prop = rm.get_pending_proposal()
    assert prop is not None
    assert prop.proposal_id == telem.active_proposal_id
    assert prop.status == ProposalStatus.PENDING_OPERATOR_REVIEW
    assert prop.proposed_route_id == "ROUTE_DIRECT_RTB"
    assert prop.destination_name == "Pune Lohegaon (Direct RTB)"
    assert len(prop.proposed_waypoints) == 3


def test_duplicate_contingency_suppression():
    rm = MissionReroutingManager()
    cands = _create_mock_candidate_routes()

    # Step 1 at t=30.0s
    state1 = _create_mock_twin_state(timestamp_s=30.0, decision_action=MissionDecisionAction.ABORT_AND_RTB)
    rm.evaluate_and_update(state1, cands)
    assert rm.get_pending_proposal_count() == 1
    first_prop_id = rm.get_pending_proposal().proposal_id

    # Step 2 at t=32.0s (within suppression duration 10.0s)
    state2 = _create_mock_twin_state(timestamp_s=32.0, decision_action=MissionDecisionAction.ABORT_AND_RTB)
    rm.evaluate_and_update(state2, cands)
    assert rm.get_pending_proposal_count() == 1
    assert rm.get_pending_proposal().proposal_id == first_prop_id


def test_pending_proposal_does_not_modify_active_route():
    rm = MissionReroutingManager()
    state = _create_mock_twin_state(timestamp_s=40.0, decision_action=MissionDecisionAction.DIVERT_TO_ALTERNATE)
    cands = _create_mock_candidate_routes()

    telem = rm.evaluate_and_update(state, cands)
    assert telem.active_route_id == "ORIGINAL_MISSION_ROUTE"
    assert rm.active_route_id == "ORIGINAL_MISSION_ROUTE"


def test_proposal_auto_expiration():
    cfg = MissionReroutingConfig(proposal_expiry_duration_s=15.0)
    rm = MissionReroutingManager(config=cfg)
    cands = _create_mock_candidate_routes()

    # Create proposal at t=50.0s
    state1 = _create_mock_twin_state(timestamp_s=50.0, decision_action=MissionDecisionAction.ABORT_AND_RTB)
    rm.evaluate_and_update(state1, cands)
    prop = rm.get_pending_proposal()
    assert prop is not None
    assert prop.status == ProposalStatus.PENDING_OPERATOR_REVIEW

    # Advance time to t=70.0s (> 15.0s expiration limit)
    state2 = _create_mock_twin_state(timestamp_s=70.0, decision_action=MissionDecisionAction.CONTINUE_MISSION)
    rm.evaluate_and_update(state2, cands)

    assert rm.get_pending_proposal() is None
    expired_prop = rm.get_proposal(prop.proposal_id)
    assert expired_prop is not None
    assert expired_prop.status == ProposalStatus.EXPIRED


def test_configuration_loading_and_validation():
    cfg = MissionReroutingConfig(
        proposal_expiry_duration_s=60.0,
        duplicate_proposal_suppression_s=5.0
    )
    rm = MissionReroutingManager(config=cfg)
    assert rm.config.proposal_expiry_duration_s == 60.0
    assert rm.config.duplicate_proposal_suppression_s == 5.0
    assert rm.config.min_twin_quality == "HEALTHY"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
