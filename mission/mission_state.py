"""
Spatial Mission and Navigation State Models
Defines typed data models for 3D Geographic Positions, Waypoints, Mission Routes,
and Mission Navigation Telemetry.
"""

from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Any


@dataclass
class GeographicPosition:
    """
    3D Geographic Position in WGS84 World Geodetic System.
    """
    latitude_deg: float      # Latitude in degrees [-90.0 to 90.0]
    longitude_deg: float     # Longitude in degrees [-180.0 to 180.0]
    altitude_m: float        # Altitude above mean sea level [m]
    timestamp_s: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class Waypoint:
    """
    Mission Navigation Waypoint.
    """
    waypoint_id: str
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    name: str = ""
    priority: int = 1

    def to_position(self) -> GeographicPosition:
        return GeographicPosition(
            latitude_deg=self.latitude_deg,
            longitude_deg=self.longitude_deg,
            altitude_m=self.altitude_m
        )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MissionRoute:
    """
    Planned Mission Flight Route.
    """
    waypoints: List[Waypoint]
    total_planned_route_distance_m: float
    current_waypoint_index: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MissionNavigationState:
    """
    State Model for Spatial Mission and Navigation.
    Tracks 3D position, waypoint progress, distances, progress %, and ETAs.
    """
    timestamp_s: float
    current_position: GeographicPosition
    home_position: GeographicPosition
    start_position: GeographicPosition
    current_waypoint: Waypoint
    current_waypoint_index: int
    total_waypoint_count: int
    distance_to_next_waypoint_m: float
    distance_to_home_m: float
    remaining_route_distance_m: float
    completed_route_distance_m: float
    total_route_distance_m: float
    mission_progress_pct: float
    estimated_ground_speed_mps: float
    eta_to_next_waypoint_s: Optional[float]
    eta_to_home_s: Optional[float]
    eta_remaining_mission_s: Optional[float]
    eta_status: str
    mission_started: bool
    mission_completed: bool
    active_route_id: str = "ORIGINAL_MISSION_ROUTE"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


# Alias for nested Digital Twin subsystem integration
MissionNavigationSubsystemState = MissionNavigationState
