"""
Spatial Mission and Navigation Engine
Implements WGS84 Haversine Great-Circle distance calculations, 3D spatial separation,
waypoint progression, route distance tracking, and safe ETA estimation.
"""

import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union, Any

from .mission_state import (
    GeographicPosition,
    Waypoint,
    MissionRoute,
    MissionNavigationState
)
from .mission_config import MissionConfig

EARTH_RADIUS_METERS = 6371000.0


def haversine_distance_m(pos1: GeographicPosition, pos2: GeographicPosition) -> float:
    """
    Calculates horizontal great-circle distance [m] between two geographic coordinates
    using the Haversine formula.
    """
    lat1 = math.radians(pos1.latitude_deg)
    lat2 = math.radians(pos2.latitude_deg)
    dlat = math.radians(pos2.latitude_deg - pos1.latitude_deg)
    dlon = math.radians(pos2.longitude_deg - pos1.longitude_deg)

    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    return EARTH_RADIUS_METERS * c


def vertical_distance_m(pos1: GeographicPosition, pos2: GeographicPosition) -> float:
    """Calculates absolute vertical altitude separation [m]."""
    return abs(pos1.altitude_m - pos2.altitude_m)


def spatial_distance_3d_m(pos1: GeographicPosition, pos2: GeographicPosition) -> float:
    """Calculates 3D spatial euclidean distance [m] combining horizontal and vertical separation."""
    d_horiz = haversine_distance_m(pos1, pos2)
    d_vert = vertical_distance_m(pos1, pos2)
    return math.sqrt(d_horiz ** 2 + d_vert ** 2)


class SpatialNavigationEngine:
    """
    Engine managing 3D spatial navigation state, waypoint arrival logic,
    route distance progress %, and ETA estimation.
    """

    def __init__(self, config: Optional[Union[MissionConfig, str, Path]] = None):
        if config is None:
            default_json = Path(__file__).resolve().parent.parent / "configs" / "mission_config.json"
            if default_json.exists():
                self.config = MissionConfig.from_json(default_json)
            else:
                self.config = MissionConfig()
        elif isinstance(config, (str, Path)):
            self.config = MissionConfig.from_json(config)
        elif isinstance(config, MissionConfig):
            self.config = config
        else:
            self.config = MissionConfig()

        self.reset()

    def reset(self):
        """Resets navigation engine to initial route state."""
        self.sim_time_s: float = 0.0
        self.current_waypoint_index: int = 0
        self.completed_route_distance_m: float = 0.0
        self.max_completed_route_distance_m: float = 0.0
        self.mission_started: bool = False
        self.mission_completed: bool = False

        self.waypoints = list(self.config.waypoints)
        self.original_waypoints = list(self.config.waypoints)
        self.completed_waypoints_history: List[Waypoint] = []
        self.route_history: List[Dict[str, Any]] = []
        self.active_route_id: str = "ORIGINAL_MISSION_ROUTE"
        self.last_route_change_timestamp_s: Optional[float] = None
        self.route_version: int = 1

        self.home_position = self.config.home_position
        self.start_position = (
            self.waypoints[0].to_position()
            if len(self.waypoints) > 0
            else self.config.home_position
        )
        self.current_position: Optional[GeographicPosition] = self.start_position

        self.total_planned_route_distance_m = self.calculate_total_route_distance(self.waypoints)

    def apply_new_route(
        self,
        new_waypoints: List[Waypoint],
        route_id: str,
        timestamp_s: float,
        current_position: Optional[GeographicPosition] = None
    ) -> bool:
        """
        Atomically applies an approved route change to the simulated navigation plan.
        Preserves current UAV position, completed waypoint history, and updates remaining distances.
        Rolls back if validation fails.
        """
        if not new_waypoints:
            raise ValueError("New route must contain at least one waypoint.")

        # Validate all new waypoints
        for wp in new_waypoints:
            if not (-90.0 <= wp.latitude_deg <= 90.0):
                raise ValueError(f"Invalid latitude {wp.latitude_deg} in waypoint {wp.waypoint_id}")
            if not (-180.0 <= wp.longitude_deg <= 180.0):
                raise ValueError(f"Invalid longitude {wp.longitude_deg} in waypoint {wp.waypoint_id}")
            if not (-100.0 <= wp.altitude_m <= 25000.0):
                raise ValueError(f"Invalid altitude {wp.altitude_m} in waypoint {wp.waypoint_id}")

        # Snapshot state for rollback safety
        rollback_state = {
            "waypoints": list(self.waypoints),
            "current_waypoint_index": self.current_waypoint_index,
            "completed_route_distance_m": self.completed_route_distance_m,
            "max_completed_route_distance_m": self.max_completed_route_distance_m,
            "total_planned_route_distance_m": self.total_planned_route_distance_m,
            "active_route_id": self.active_route_id,
            "completed_waypoints_history": list(self.completed_waypoints_history),
            "mission_completed": self.mission_completed,
            "last_route_change_timestamp_s": self.last_route_change_timestamp_s,
            "route_version": self.route_version,
        }

        try:
            # 1. Preserve completed waypoints history
            visited = self.waypoints[:self.current_waypoint_index]
            for wp in visited:
                if wp not in self.completed_waypoints_history:
                    self.completed_waypoints_history.append(wp)

            # 2. Formulate combined waypoints sequence
            if len(visited) > 0:
                self.waypoints = list(visited) + list(new_waypoints)
                self.current_waypoint_index = len(visited)
            else:
                self.waypoints = list(new_waypoints)
                self.current_waypoint_index = 0

            # 3. Recalculate total planned route distance
            self.total_planned_route_distance_m = self.calculate_total_route_distance(self.waypoints)
            self.mission_completed = False
            self.last_route_change_timestamp_s = float(timestamp_s)
            self.active_route_id = str(route_id)
            self.route_version += 1

            self.route_history.append({
                "route_id": self.active_route_id,
                "timestamp_s": self.last_route_change_timestamp_s,
                "applied_waypoint_count": len(new_waypoints),
                "total_waypoints": len(self.waypoints),
                "current_waypoint_index": self.current_waypoint_index,
                "total_distance_m": self.total_planned_route_distance_m,
                "route_version": self.route_version
            })
            return True
        except Exception as e:
            # Rollback to pre-change snapshot
            self.waypoints = rollback_state["waypoints"]
            self.current_waypoint_index = rollback_state["current_waypoint_index"]
            self.completed_route_distance_m = rollback_state["completed_route_distance_m"]
            self.max_completed_route_distance_m = rollback_state["max_completed_route_distance_m"]
            self.total_planned_route_distance_m = rollback_state["total_planned_route_distance_m"]
            self.active_route_id = rollback_state["active_route_id"]
            self.completed_waypoints_history = rollback_state["completed_waypoints_history"]
            self.mission_completed = rollback_state["mission_completed"]
            self.last_route_change_timestamp_s = rollback_state["last_route_change_timestamp_s"]
            self.route_version = rollback_state["route_version"]
            raise RuntimeError(f"Atomic route application failed and was rolled back: {e}") from e

    @staticmethod
    def calculate_total_route_distance(waypoints: List[Waypoint]) -> float:
        """Calculates total planned route distance [m] along ordered waypoints."""
        if len(waypoints) < 2:
            return 0.0
        
        total = 0.0
        for i in range(len(waypoints) - 1):
            total += haversine_distance_m(waypoints[i].to_position(), waypoints[i + 1].to_position())
        return total

    @property
    def remaining_route_distance_m(self) -> float:
        """Remaining distance along planned route [m]."""
        return max(0.0, self.total_planned_route_distance_m - self.completed_route_distance_m)

    @property
    def mission_progress_pct(self) -> float:
        """Mission progress percentage [0 to 100%]."""
        if self.total_planned_route_distance_m <= 0.0:
            return 100.0
        return min(100.0, max(0.0, (self.completed_route_distance_m / self.total_planned_route_distance_m) * 100.0))

    def update(
        self,
        current_position: GeographicPosition,
        ground_speed_mps: float = 55.0,
        timestamp_s: float = 0.0
    ) -> MissionNavigationState:
        """
        Updates navigation state given UAV current 3D position and ground speed.

        :param current_position: UAV WGS84 position (lat, lon, alt)
        :param ground_speed_mps: Current ground speed [m/s]
        :param timestamp_s: Current simulation timestamp [s]
        """
        self.sim_time_s = float(timestamp_s)
        self.mission_started = True
        self.current_position = current_position

        if len(self.waypoints) == 0:
            # Empty waypoints edge case
            return MissionNavigationState(
                timestamp_s=self.sim_time_s,
                current_position=current_position,
                home_position=self.home_position,
                start_position=self.start_position,
                current_waypoint=Waypoint("WP-NONE", self.home_position.latitude_deg, self.home_position.longitude_deg, self.home_position.altitude_m, "No Waypoint"),
                current_waypoint_index=0,
                total_waypoint_count=0,
                distance_to_next_waypoint_m=0.0,
                distance_to_home_m=haversine_distance_m(current_position, self.home_position),
                remaining_route_distance_m=0.0,
                completed_route_distance_m=0.0,
                total_route_distance_m=0.0,
                mission_progress_pct=100.0,
                estimated_ground_speed_mps=ground_speed_mps,
                eta_to_next_waypoint_s=None,
                eta_to_home_s=None,
                eta_remaining_mission_s=None,
                eta_status="UNAVAILABLE_EMPTY_ROUTE",
                mission_started=True,
                mission_completed=True
            )

        # 1. Active Waypoint Logic & Progression
        current_wp = self.waypoints[self.current_waypoint_index]
        d_next_m = haversine_distance_m(current_position, current_wp.to_position())
        d_home_m = haversine_distance_m(current_position, self.home_position)

        # Advance to next waypoint if within arrival radius
        if d_next_m <= self.config.waypoint_arrival_radius_m:
            if self.current_waypoint_index < len(self.waypoints) - 1:
                self.current_waypoint_index += 1
                current_wp = self.waypoints[self.current_waypoint_index]
                d_next_m = haversine_distance_m(current_position, current_wp.to_position())
            else:
                self.mission_completed = True

        # Check final waypoint reach
        if self.current_waypoint_index == len(self.waypoints) - 1 and d_next_m <= self.config.waypoint_arrival_radius_m:
            self.mission_completed = True

        # 2. Distance Progress % Calculation
        total_route_m = self.total_planned_route_distance_m

        if self.mission_completed or total_route_m < 1.0:
            completed_dist_m = total_route_m
            remaining_dist_m = 0.0
            progress_pct = 100.0
        else:
            # Sum completed segments 0->1, 1->2 ... (k-1)->k
            completed_segs_m = 0.0
            for i in range(self.current_waypoint_index):
                completed_segs_m += haversine_distance_m(self.waypoints[i].to_position(), self.waypoints[i + 1].to_position())

            # Progress along active segment k->k+1
            k = self.current_waypoint_index
            if k == 0:
                dist_from_start = haversine_distance_m(self.start_position, current_position)
                segment_progress_m = min(haversine_distance_m(self.start_position, self.waypoints[0].to_position()), dist_from_start)
            else:
                prev_wp_pos = self.waypoints[k - 1].to_position()
                curr_wp_pos = self.waypoints[k].to_position()
                seg_len = haversine_distance_m(prev_wp_pos, curr_wp_pos)
                dist_from_prev = haversine_distance_m(prev_wp_pos, current_position)
                segment_progress_m = min(seg_len, dist_from_prev)

            current_calc_completed = completed_segs_m + segment_progress_m
            
            # Monotonic non-decreasing progress constraint
            self.max_completed_route_distance_m = max(self.max_completed_route_distance_m, current_calc_completed)
            completed_dist_m = min(total_route_m, self.max_completed_route_distance_m)
            self.completed_route_distance_m = completed_dist_m
            remaining_dist_m = max(0.0, total_route_m - completed_dist_m)
            progress_pct = min(100.0, max(0.0, (completed_dist_m / total_route_m) * 100.0))

        # 3. Ground Speed & ETA Calculations
        speed = max(0.0, float(ground_speed_mps))

        if speed < 1.0:
            eta_next = None
            eta_home = None
            eta_remaining = None
            eta_status = "UNAVAILABLE_LOW_SPEED"
        else:
            eta_next = round(d_next_m / speed, 1)
            eta_home = round(d_home_m / speed, 1)
            eta_remaining = round(remaining_dist_m / speed, 1)
            eta_status = "AVAILABLE"

        return MissionNavigationState(
            timestamp_s=round(self.sim_time_s, 2),
            current_position=current_position,
            home_position=self.home_position,
            start_position=self.start_position,
            current_waypoint=current_wp,
            current_waypoint_index=self.current_waypoint_index,
            total_waypoint_count=len(self.waypoints),
            distance_to_next_waypoint_m=round(d_next_m, 1),
            distance_to_home_m=round(d_home_m, 1),
            remaining_route_distance_m=round(remaining_dist_m, 1),
            completed_route_distance_m=round(completed_dist_m, 1),
            total_route_distance_m=round(total_route_m, 1),
            mission_progress_pct=round(progress_pct, 2),
            estimated_ground_speed_mps=round(speed, 1),
            eta_to_next_waypoint_s=eta_next,
            eta_to_home_s=eta_home,
            eta_remaining_mission_s=eta_remaining,
            eta_status=eta_status,
            mission_started=self.mission_started,
            mission_completed=self.mission_completed,
            active_route_id=self.active_route_id
        )
