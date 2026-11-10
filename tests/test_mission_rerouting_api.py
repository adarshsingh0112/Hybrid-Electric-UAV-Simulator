"""
API Integration Tests for Mission Rerouting & Operator Confirmation (Step 10)
Uses FastAPI TestClient to test route query, pending proposal polling,
pilot approval/rejection endpoints, and audit logging.
"""
import pytest
from fastapi.testclient import TestClient
from engine_service import app, twin_runtime
from mission.decision_engine import MissionDecisionAction, MissionDecisionResult
from mission.trajectory_optimizer import CandidateRoute
from mission.mission_state import GeographicPosition, Waypoint


@pytest.fixture(autouse=True)
def reset_runtime():
    twin_runtime.reset()
    yield
    twin_runtime.reset()


def _inject_test_proposal():
    twin = twin_runtime.uav_digital_twin
    state = twin.latest_state
    if not state:
        twin_runtime.step(dt_s=0.05)
        state = twin.latest_state

    # Inject decision requiring RTB
    state.decision = MissionDecisionResult(
        recommended_action=MissionDecisionAction.ABORT_AND_RTB.value,
        decision_reason="Engine thermal overheat contingency",
        decision_confidence=0.98,
        triggered_conditions=["ENGINE_OVERHEAT_220C"],
        mission_feasible=False,
        rtb_feasible=True,
        alternate_feasible=True,
        selected_destination="Pune Lohegaon (Direct RTB)",
        decision_timestamp=state.timestamp_s,
        data_quality_status="HEALTHY"
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
        total_distance_m=92000.0,
        total_duration_s=1650.0,
        total_fuel_liters=12.5,
        total_battery_energy_wh=1900.0,
        remaining_fuel_liters=37.5,
        remaining_battery_soc_pct=78.0,
        reserve_margin_pct=26.0,
        is_feasible=True,
        multi_criteria_cost=10.2,
        selection_rank=1
    )

    twin.rerouting_manager.evaluate_and_update(state, [cand])
    return twin.rerouting_manager.get_pending_proposal()


def test_get_mission_route_endpoint():
    client = TestClient(app)
    response = client.get("/api/mission/route")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "SUCCESS"
    assert "active_route_id" in data
    assert "waypoints" in data
    assert len(data["waypoints"]) > 0
    assert data["active_route_id"] == "ORIGINAL_MISSION_ROUTE"


def test_get_mission_proposals_endpoint():
    client = TestClient(app)
    response = client.get("/api/mission/proposals")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "SUCCESS"
    assert "proposals" in data
    assert "pending_count" in data


def test_get_pending_proposal_endpoint():
    client = TestClient(app)
    # Initially no pending proposal
    res1 = client.get("/api/mission/proposals/pending")
    assert res1.status_code == 200
    assert res1.json()["has_pending"] is False

    # Inject proposal
    prop = _inject_test_proposal()
    assert prop is not None

    res2 = client.get("/api/mission/proposals/pending")
    assert res2.status_code == 200
    d2 = res2.json()
    assert d2["has_pending"] is True
    assert d2["proposal"]["proposal_id"] == prop.proposal_id
    assert d2["proposal"]["proposed_route_id"] == "ROUTE_DIRECT_RTB"


def test_approve_proposal_endpoint():
    client = TestClient(app)
    prop = _inject_test_proposal()
    assert prop is not None

    # Unknown ID should fail 400
    bad_res = client.post("/api/mission/proposals/UNKNOWN_999/approve", json={"operator_id": "PILOT-GCS-01"})
    assert bad_res.status_code == 400

    # Valid ID succeeds
    good_res = client.post(f"/api/mission/proposals/{prop.proposal_id}/approve", json={"operator_id": "PILOT-GCS-01"})
    assert good_res.status_code == 200
    d = good_res.json()
    assert d["status"] == "SUCCESS"
    assert d["active_route_id"] == "ROUTE_DIRECT_RTB"

    # Route endpoint now reflects updated route
    route_res = client.get("/api/mission/route")
    assert route_res.json()["active_route_id"] == "ROUTE_DIRECT_RTB"


def test_reject_proposal_endpoint():
    client = TestClient(app)
    prop = _inject_test_proposal()
    assert prop is not None

    rej_res = client.post(
        f"/api/mission/proposals/{prop.proposal_id}/reject",
        json={"operator_id": "PILOT-GCS-01", "reason": "Weather diversion not required"}
    )
    assert rej_res.status_code == 200
    d = rej_res.json()
    assert d["status"] == "SUCCESS"
    assert "rejected" in d["message"].lower()

    # Active route remains unchanged
    route_res = client.get("/api/mission/route")
    assert route_res.json()["active_route_id"] == "ORIGINAL_MISSION_ROUTE"


def test_audit_trail_endpoint():
    client = TestClient(app)
    prop = _inject_test_proposal()
    client.post(f"/api/mission/proposals/{prop.proposal_id}/reject", json={"operator_id": "PILOT-GCS-01", "reason": "Test"})

    res = client.get("/api/mission/audit")
    assert res.status_code == 200
    data = res.json()
    assert data["status"] == "SUCCESS"
    assert len(data["audit_events"]) > 0

    # Verify event fields
    ev = data["audit_events"][-1]
    assert "event_id" in ev
    assert "timestamp_s" in ev
    assert "operator_action" in ev


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
