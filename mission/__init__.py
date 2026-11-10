"""
DRDO MALE UAV Spatial Mission and Navigation Package
3D Geographic Positions, Waypoint Routes, Navigation Engine, and Progress Tracking.
"""

from .mission_state import (
    GeographicPosition,
    Waypoint,
    MissionRoute,
    MissionNavigationState,
    MissionNavigationSubsystemState,
)
from .mission_config import MissionConfig
from .navigation import (
    haversine_distance_m,
    vertical_distance_m,
    spatial_distance_3d_m,
    SpatialNavigationEngine,
)

from .reliability_engine import (
    ContingencyAction,
    ReliabilityStatusLevel,
    AlternateAirfield,
    ReliabilityConfig,
    RTBPredictionResult,
    ReliabilityScoreBreakdown,
    MissionReliabilityState,
    MissionReliabilityEngine,
)

from .decision_engine import (
    MissionDecisionAction,
    DestinationInfo,
    DecisionEngineConfig,
    MissionDecisionResult,
    MissionDecisionEngine,
)

from .trajectory_optimizer import (
    PropulsionStrategy,
    CandidateRouteSegment,
    CandidateRoute,
    TrajectoryOptimizationTelemetry,
    TrajectoryOptimizer,
)

from .predictive_risk_analyzer import (
    compute_percentile,
    compute_wilson_score_interval,
    RouteRiskSummary,
    PredictiveRiskReport,
    PredictiveSimulationTelemetry,
    PredictiveRiskAnalyzer,
)

from .mission_rerouting_manager import (
    ProposalStatus,
    RouteProposal,
    AuditEvent,
    MissionReroutingTelemetry,
    MissionReroutingConfig,
    MissionReroutingManager,
)

__all__ = [
    "GeographicPosition",
    "Waypoint",
    "MissionRoute",
    "MissionNavigationState",
    "MissionNavigationSubsystemState",
    "MissionConfig",
    "haversine_distance_m",
    "vertical_distance_m",
    "spatial_distance_3d_m",
    "SpatialNavigationEngine",
    "ContingencyAction",
    "ReliabilityStatusLevel",
    "AlternateAirfield",
    "ReliabilityConfig",
    "RTBPredictionResult",
    "ReliabilityScoreBreakdown",
    "MissionReliabilityState",
    "MissionReliabilityEngine",
    "MissionDecisionAction",
    "DestinationInfo",
    "DecisionEngineConfig",
    "MissionDecisionResult",
    "MissionDecisionEngine",
    "PropulsionStrategy",
    "CandidateRouteSegment",
    "CandidateRoute",
    "TrajectoryOptimizationTelemetry",
    "TrajectoryOptimizer",
    "compute_percentile",
    "compute_wilson_score_interval",
    "RouteRiskSummary",
    "PredictiveRiskReport",
    "PredictiveSimulationTelemetry",
    "PredictiveRiskAnalyzer",
    "ProposalStatus",
    "RouteProposal",
    "AuditEvent",
    "MissionReroutingTelemetry",
    "MissionReroutingConfig",
    "MissionReroutingManager",
]

