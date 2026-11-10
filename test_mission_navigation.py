"""
Spatial Mission and Navigation Test Suite (Step 5)
Tests:
1. Haversine & 3D Geographic Distance Calculations against known benchmarks.
2. Waypoint progression (UAV reaching waypoint advances active waypoint).
3. Route distance mission progress % calculation (monotonically non-decreasing, bounded [0, 100]).
4. Distance to home base tracking.
5. Final waypoint arrival (marks mission_completed = True, progress = 100%).
6. Invalid geographic data validation (catches out-of-range lat/lon/alt).
7. Timestamp synchronization & stale data detection.
8. Zero/Low Ground Speed ETA safety (no division by zero, safe status).
"""

import sys
from pathlib import Path
import math

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from mission.mission_state import GeographicPosition, Waypoint
from mission.mission_config import MissionConfig
from mission.navigation import (
    haversine_distance_m,
    vertical_distance_m,
    spatial_distance_3d_m,
    SpatialNavigationEngine
)
from digital_twin.uav_digital_twin import (
    UAVDigitalTwin,
    DigitalTwinQualityStatus,
    UAVDigitalTwinValidator
)
from simulation.flight_profile import FlightProfile, FlightState
from simulation.hybrid_powertrain import HybridPowertrain


def test_haversine_distance():
    print("[TEST 1] Testing Haversine & 3D Distance Calculations...")
    # Known Benchmark Coordinates:
    # Lohegaon Base (18.5822° N, 73.9197° E) to Waypoint 1 (18.8000° N, 74.1000° E)
    pos_base = GeographicPosition(18.5822, 73.9197, 560.0)
    pos_wp1 = GeographicPosition(18.8000, 74.1000, 5000.0)

    d_horiz = haversine_distance_m(pos_base, pos_wp1)
    d_vert = vertical_distance_m(pos_base, pos_wp1)
    d_3d = spatial_distance_3d_m(pos_base, pos_wp1)

    print(f"  -> Horizontal Distance: {d_horiz/1000.0:.2f} km ({d_horiz:.1f} m)")
    print(f"  -> Vertical Separation: {d_vert:.1f} m")
    print(f"  -> 3D Spatial Distance:  {d_3d/1000.0:.2f} km ({d_3d:.1f} m)")

    # Expected horizontal distance ~30.0 km
    assert 28000.0 < d_horiz < 32000.0, f"Unexpected haversine distance: {d_horiz} m"
    assert abs(d_vert - 4440.0) < 1.0, f"Unexpected vertical distance: {d_vert} m"
    assert d_3d > d_horiz, "3D spatial distance must be greater than horizontal distance"
    print("  -> PASSED: Geographic distance calculation matches physical benchmarks.")


def test_waypoint_progression():
    print("\n[TEST 2] Testing Waypoint Progression Logic...")
    nav = SpatialNavigationEngine()
    
    # Initial active waypoint: Waypoint 0 (18.6500, 73.9500)
    assert nav.current_waypoint_index == 0
    wp0 = nav.waypoints[0]
    print(f"  -> Initial Active Waypoint: {wp0.waypoint_id} ({wp0.name})")

    # Fly UAV right to Waypoint 0 location (within 250m arrival radius)
    pos_at_wp0 = GeographicPosition(18.6500, 73.9500, 1500.0)
    state = nav.update(pos_at_wp0, ground_speed_mps=55.0, timestamp_s=10.0)

    print(f"  -> Position at WP0 -> Advanced to Active Waypoint Index: {state.current_waypoint_index} ({state.current_waypoint.waypoint_id})")

    assert state.current_waypoint_index == 1, "Failed to advance waypoint index after arrival"
    assert state.current_waypoint.waypoint_id == "WP-01", "Expected WP-01 as active waypoint"
    print("  -> PASSED: Waypoint progression advances correctly upon arrival.")


def test_mission_progress_pct():
    print("\n[TEST 3] Testing Mission Progress % Calculation...")
    nav = SpatialNavigationEngine()
    
    initial_state = nav.update(nav.start_position, ground_speed_mps=55.0, timestamp_s=0.0)
    print(f"  -> Total Planned Route Distance: {nav.total_planned_route_distance_m/1000.0:.2f} km")
    print(f"  -> Start Position Progress: {initial_state.mission_progress_pct:.2f}%")

    # Move along waypoints sequentially
    prev_progress = initial_state.mission_progress_pct
    for idx in range(len(nav.waypoints)):
        wp_pos = nav.waypoints[idx].to_position()
        st = nav.update(wp_pos, ground_speed_mps=55.0, timestamp_s=(idx + 1) * 100.0)
        print(f"  -> At {nav.waypoints[idx].waypoint_id}: Completed={st.completed_route_distance_m/1000.0:.1f} km, Progress={st.mission_progress_pct:.2f}%")
        
        assert st.mission_progress_pct >= prev_progress, "Mission progress % must be monotonically non-decreasing"
        assert 0.0 <= st.mission_progress_pct <= 100.0, "Progress % must be bounded between 0 and 100%"
        prev_progress = st.mission_progress_pct

    print("  -> PASSED: Mission progress calculation is monotonically non-decreasing and bounded [0, 100].")


def test_home_distance():
    print("\n[TEST 4] Testing Home Distance Updating...")
    nav = SpatialNavigationEngine()
    
    # At home base
    st_home = nav.update(nav.home_position, ground_speed_mps=0.0, timestamp_s=0.0)
    print(f"  -> Distance to Home (at Base): {st_home.distance_to_home_m:.1f} m")
    assert st_home.distance_to_home_m < 1.0, "Distance to home at home base should be ~0m"

    # Move 50 km away
    pos_far = GeographicPosition(19.0000, 74.3000, 5000.0)
    st_far = nav.update(pos_far, ground_speed_mps=55.0, timestamp_s=500.0)
    print(f"  -> Distance to Home (50km out): {st_far.distance_to_home_m/1000.0:.2f} km ({st_far.distance_to_home_m:.1f} m)")
    
    assert st_far.distance_to_home_m > 40000.0, "Home distance should increase as UAV moves away"
    print("  -> PASSED: Distance to home base updates accurately.")


def test_final_waypoint_arrival():
    print("\n[TEST 5] Testing Final Waypoint Arrival & Mission Completion...")
    nav = SpatialNavigationEngine()
    
    # Move directly to the final waypoint (WP-06)
    final_wp = nav.waypoints[-1]
    final_pos = final_wp.to_position()

    # Step through route to final waypoint
    for idx in range(len(nav.waypoints)):
        st = nav.update(nav.waypoints[idx].to_position(), ground_speed_mps=55.0, timestamp_s=(idx+1)*200.0)

    # Final update at last waypoint
    st_final = nav.update(final_pos, ground_speed_mps=55.0, timestamp_s=1500.0)

    print(f"  -> Final Waypoint ({final_wp.waypoint_id}): Completed={st_final.completed_route_distance_m/1000.0:.2f} km / {st_final.total_route_distance_m/1000.0:.2f} km")
    print(f"  -> Mission Progress: {st_final.mission_progress_pct:.2f}% | Mission Completed: {st_final.mission_completed}")

    assert st_final.mission_completed == True, "Final waypoint arrival should mark mission_completed = True"
    assert st_final.mission_progress_pct == 100.0, "Final mission progress % should be 100.0%"
    assert st_final.remaining_route_distance_m == 0.0, "Remaining route distance should be 0.0 m"
    print("  -> PASSED: Final waypoint arrival marks mission completion at 100%.")


def test_invalid_geographic_data():
    print("\n[TEST 6] Testing Invalid Geographic Data Validation...")
    powertrain = HybridPowertrain()
    digital_twin = UAVDigitalTwin()
    flight = FlightProfile().get_standard_mission_state(100.0, 3600.0)

    hybrid_telemetry = powertrain.update(50000.0, flight, 15.0, 0.05)
    
    # Inject invalid latitude (95.0 degrees N)
    invalid_pos = GeographicPosition(latitude_deg=95.0, longitude_deg=73.9197, altitude_m=500.0)
    snapshot = digital_twin.update_from_hybrid_telemetry(hybrid_telemetry, flight, current_position=invalid_pos)

    print(f"  -> Injected Latitude: 95.0° N | Digital Twin Quality Status: {snapshot.quality_status}")
    print(f"  -> Validation Errors Detected: {snapshot.validation_errors}")

    assert snapshot.quality_status == DigitalTwinQualityStatus.INVALID.value
    assert len(snapshot.validation_errors) > 0
    assert any("Latitude out of physical geographic bounds" in err for err in snapshot.validation_errors)
    print("  -> PASSED: Invalid geographic data caught and marked INVALID.")


def test_timestamp_synchronization():
    print("\n[TEST 7] Testing Navigation Timestamp Synchronization...")
    powertrain = HybridPowertrain()
    digital_twin = UAVDigitalTwin()
    flight = FlightProfile().get_standard_mission_state(100.0, 3600.0)

    hybrid_telemetry = powertrain.update(50000.0, flight, 15.0, 0.05)
    snapshot = digital_twin.update_from_hybrid_telemetry(hybrid_telemetry, flight)

    # Manually corrupt mission subsystem timestamp to simulate 1.5s lag
    uav_state = digital_twin.latest_state
    uav_state.mission.timestamp_s = uav_state.timestamp_s - 1.5

    status = UAVDigitalTwinValidator.validate(uav_state, stale_timeout_s=0.20)

    print(f"  -> Lag Injected: 1.5s | Quality Status: {uav_state.quality_status}")
    print(f"  -> Stale Flags: {uav_state.stale_data_flags}")

    assert uav_state.stale_data_flags["mission"] == True
    assert uav_state.quality_status == DigitalTwinQualityStatus.DEGRADED
    print("  -> PASSED: Stale navigation telemetry flagged DEGRADED correctly.")


def test_zero_speed_eta_safety():
    print("\n[TEST 8] Testing Zero/Low Speed ETA Safety...")
    nav = SpatialNavigationEngine()
    pos = GeographicPosition(18.7000, 74.0000, 2000.0)

    # Zero ground speed
    st_zero = nav.update(pos, ground_speed_mps=0.0, timestamp_s=10.0)
    print(f"  -> Zero Speed ETA Status: {st_zero.eta_status}")
    print(f"  -> ETA to Next WP: {st_zero.eta_to_next_waypoint_s} | ETA to Home: {st_zero.eta_to_home_s}")

    assert st_zero.eta_status == "UNAVAILABLE_LOW_SPEED"
    assert st_zero.eta_to_next_waypoint_s is None
    assert st_zero.eta_to_home_s is None
    assert st_zero.eta_remaining_mission_s is None

    # Normal ground speed (55 m/s)
    st_normal = nav.update(pos, ground_speed_mps=55.0, timestamp_s=20.0)
    print(f"  -> 55 m/s Speed ETA Status: {st_normal.eta_status}")
    print(f"  -> ETA to Next WP: {st_normal.eta_to_next_waypoint_s} s | ETA to Home: {st_normal.eta_to_home_s} s")

    assert st_normal.eta_status == "AVAILABLE"
    assert st_normal.eta_to_next_waypoint_s > 0.0
    assert st_normal.eta_to_home_s > 0.0
    print("  -> PASSED: Zero/Low speed handled safely without division by zero.")


if __name__ == "__main__":
    print("=" * 75)
    print("  DRDO MALE UAV SPATIAL MISSION & NAVIGATION TEST SUITE (STEP 5)")
    print("=" * 75)
    test_haversine_distance()
    test_waypoint_progression()
    test_mission_progress_pct()
    test_home_distance()
    test_final_waypoint_arrival()
    test_invalid_geographic_data()
    test_timestamp_synchronization()
    test_zero_speed_eta_safety()
    print("\n" + "=" * 75)
    print("ALL SPATIAL MISSION & NAVIGATION TESTS PASSED SUCCESSFULLY! (8/8 PASS)")
    print("=" * 75)
