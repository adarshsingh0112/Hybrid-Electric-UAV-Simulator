"""
Configuration Loader for Spatial Mission and Navigation.
"""

from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Dict, List, Optional, Union, Any

from .mission_state import GeographicPosition, Waypoint


@dataclass
class MissionConfig:
    """
    Configuration specification for Spatial Mission Navigation.
    """
    mission_name: str = "DRDO-MALE-UAV-Surveillance-Sortie-Alpha"
    waypoint_arrival_radius_m: float = 250.0 # Waypoint arrival radius [m]
    default_ground_speed_mps: float = 55.0   # Default nominal ground speed [m/s]
    home_position: GeographicPosition = field(
        default_factory=lambda: GeographicPosition(18.5822, 73.9197, 560.0)
    )
    waypoints: List[Waypoint] = field(
        default_factory=lambda: [
            Waypoint("WP-00", 18.6500, 73.9500, 1500.0, "Takeoff & Climb Out", 1),
            Waypoint("WP-01", 18.8000, 74.1000, 5000.0, "Enroute Climb Point", 1),
            Waypoint("WP-02", 19.1000, 74.4000, 8686.8, "High Alt Loiter Sector Alpha", 2),
            Waypoint("WP-03", 19.4000, 74.7000, 8686.8, "Surveillance Sector Bravo", 2),
            Waypoint("WP-04", 19.6000, 74.9000, 8686.8, "Recon Target Waypoint", 3),
            Waypoint("WP-05", 18.9000, 74.2000, 2000.0, "Return Descent Waypoint", 1),
            Waypoint("WP-06", 18.5822, 73.9197, 560.0, "Base Touchdown & Recover", 1),
        ]
    )

    @classmethod
    def from_json(cls, json_path: Union[str, Path]) -> "MissionConfig":
        """Loads mission configuration from a JSON file."""
        path = Path(json_path)
        if not path.exists():
            raise FileNotFoundError(f"Mission configuration file not found: {path}")

        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        home_pos = GeographicPosition(**data["home_position"])
        wps = [Waypoint(**wp) for wp in data.get("waypoints", [])]

        return cls(
            mission_name=data.get("mission_name", "DRDO-MALE-UAV-Surveillance-Sortie-Alpha"),
            waypoint_arrival_radius_m=float(data.get("waypoint_arrival_radius_m", 250.0)),
            default_ground_speed_mps=float(data.get("default_ground_speed_mps", 55.0)),
            home_position=home_pos,
            waypoints=wps
        )
