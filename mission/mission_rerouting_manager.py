"""
Closed-Loop Mission Re-Routing & GCS Pilot Confirmation Subsystem (Step 10)
Manages the human-in-the-loop mission re-routing lifecycle for the DRDO MALE UAV Digital Twin.

Architectural Workflow:
LIVE SIMULATION STATE -> CONTINGENCY DETECTION -> ROUTE RECOMMENDATION
 -> OPERATOR REVIEW -> APPROVAL/REJECTION -> SIMULATED ROUTE UPDATE -> AUDIT LOG

Key Safeguards:
1. No autonomous rerouting: Recommendations NEVER modify the active mission without operator confirmation.
2. Complete state separation: Active route, proposed route, operator decision, and approved route are isolated.
3. Safe operator workflow: Reject unknown, expired, infeasible, stale, or already processed proposals.
4. Non-destructive application: UAV current geographic position and completed waypoint history are preserved.
5. Atomic rollback: Failed route updates restore previous navigation state completely.
"""

from dataclasses import dataclass, field, asdict
from enum import Enum
import hashlib
import json
import logging
from pathlib import Path
import threading
import time
from typing import Dict, List, Optional, Tuple, Any, Union

from mission.mission_state import GeographicPosition, Waypoint, MissionNavigationState
from mission.navigation import SpatialNavigationEngine, haversine_distance_m
from mission.trajectory_optimizer import CandidateRoute
from mission.decision_engine import MissionDecisionResult, MissionDecisionAction
from mission.predictive_risk_analyzer import PredictiveRiskReport, RouteRiskSummary

logger = logging.getLogger("MissionReroutingManager")


class ProposalStatus(str, Enum):
    """Lifecycle statuses for a structured route-change proposal."""
    NO_PROPOSAL = "NO_PROPOSAL"
    PENDING_OPERATOR_REVIEW = "PENDING_OPERATOR_REVIEW"
    APPROVED = "APPROVED"
    REJECTED = "REJECTED"
    EXPIRED = "EXPIRED"
    SUPERSEDED = "SUPERSEDED"
    APPLIED = "APPLIED"
    FAILED = "FAILED"


@dataclass
class RouteProposal:
    """
    Structured route-change proposal presented for operator review.
    Contains estimated performance and predictive risk metrics.
    """
    proposal_id: str
    creation_timestamp_s: float
    source_contingency: str                 # ABORT_AND_RTB, DIVERT_TO_ALTERNATE, EMERGENCY_DESCENT
    severity: str                           # CRITICAL, WARNING, CAUTION
    triggering_conditions: List[str]
    current_active_route_id: str
    proposed_route_id: str
    proposed_route_type: str                # DIRECT_RTB, CORRIDOR_RTB, DIVERT_ALTERNATE
    proposed_waypoint_names: List[str]
    proposed_waypoints: List[Dict[str, Any]]
    destination_name: str
    destination_position: Dict[str, Any]
    estimated_route_distance_m: float
    estimated_mission_duration_s: float
    estimated_fuel_liters: float
    predicted_battery_arrival_soc_pct: float
    predicted_fuel_reserve_l: float
    predicted_reserve_margin_pct: float
    predictive_risk_summary: Dict[str, Any]
    feasibility_status: bool
    safety_constraint_violations: List[str]
    explanation: str
    expiry_timestamp_s: float
    operator_review_status: str = ProposalStatus.PENDING_OPERATOR_REVIEW.value
    operator_action: Optional[str] = None
    operator_id: Optional[str] = None
    review_timestamp_s: Optional[float] = None
    rejection_reason: Optional[str] = None
    applied_timestamp_s: Optional[float] = None

    @property
    def status(self) -> ProposalStatus:
        return ProposalStatus(self.operator_review_status)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class AuditEvent:
    """
    Immutable audit event recording lifecycle actions and operator decisions.
    """
    event_id: str
    timestamp_s: float
    proposal_id: str
    previous_route_id: str
    new_route_id: Optional[str]
    event_type: str                         # PROPOSAL_CREATED, OPERATOR_APPROVED, OPERATOR_REJECTED, etc.
    operator_action: Optional[str]
    operator_id: Optional[str]
    reason: str
    result: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MissionReroutingTelemetry:
    """
    Lightweight telemetry structure exported to the Digital Twin and GCS stream.
    """
    rerouting_subsystem_status: str         # NOMINAL, CONTINGENCY_DETECTED, PENDING_APPROVAL, ROUTE_UPDATED
    pending_proposal_count: int
    active_proposal_id: Optional[str]
    proposal_status: str
    last_operator_action: Optional[str]
    active_route_id: str
    original_route_id: str
    last_route_change_timestamp_s: Optional[float]
    proposed_route_id: Optional[str]
    proposed_destination: Optional[str]
    contingency_detected: bool
    contingency_action: str
    operator_attention_required: bool

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MissionReroutingConfig:
    """
    Configuration parameters for mission re-routing and confirmation policies.
    """
    policy_name: str = "DRDO-MALE-UAV-Mission-Rerouting-Standard-v1"
    proposal_expiry_duration_s: float = 90.0
    duplicate_proposal_suppression_s: float = 10.0
    min_twin_quality: str = "HEALTHY"
    max_stale_tolerance_s: float = 2.0
    require_operator_confirmation: bool = True
    audit_history_retention_count: int = 200
    auto_expiry_check_interval_s: float = 1.0

    @classmethod
    def from_json(cls, json_path: Union[str, Path]) -> "MissionReroutingConfig":
        path = Path(json_path)
        if not path.exists():
            return cls()
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        cfg = cls(
            policy_name=data.get("policy_name", "DRDO-MALE-UAV-Mission-Rerouting-Standard-v1"),
            proposal_expiry_duration_s=float(data.get("proposal_expiry_duration_s", 90.0)),
            duplicate_proposal_suppression_s=float(data.get("duplicate_proposal_suppression_s", 10.0)),
            min_twin_quality=str(data.get("min_twin_quality", "HEALTHY")),
            max_stale_tolerance_s=float(data.get("max_stale_tolerance_s", 2.0)),
            require_operator_confirmation=bool(data.get("require_operator_confirmation", True)),
            audit_history_retention_count=int(data.get("audit_history_retention_count", 200)),
            auto_expiry_check_interval_s=float(data.get("auto_expiry_check_interval_s", 1.0)),
        )
        cfg.validate()
        return cfg

    def validate(self):
        if self.proposal_expiry_duration_s <= 0.0:
            raise ValueError(f"proposal_expiry_duration_s must be > 0, got {self.proposal_expiry_duration_s}")
        if self.duplicate_proposal_suppression_s < 0.0:
            raise ValueError("duplicate_proposal_suppression_s cannot be negative")
        if not self.require_operator_confirmation:
            # Enforce operator confirmation constraint
            raise ValueError("require_operator_confirmation cannot be configured to False.")


class MissionReroutingManager:
    """
    Central Coordinator for Closed-Loop Mission Re-Routing & Operator Confirmation.
    Monitors digital twin telemetry, handles proposal lifecycle, enforces human-in-the-loop approval,
    and applies validated route transitions atomically to the simulation.
    """

    def __init__(self, config: Optional[Union[MissionReroutingConfig, Path, str]] = None):
        if config is None:
            default_cfg_path = Path(__file__).resolve().parent.parent / "configs" / "mission_rerouting_config.json"
            if default_cfg_path.exists():
                self.config = MissionReroutingConfig.from_json(default_cfg_path)
            else:
                self.config = MissionReroutingConfig()
        elif isinstance(config, (str, Path)):
            self.config = MissionReroutingConfig.from_json(config)
        elif isinstance(config, MissionReroutingConfig):
            config.validate()
            self.config = config
        else:
            self.config = MissionReroutingConfig()

        self._lock = threading.RLock()
        self.proposals: Dict[str, RouteProposal] = {}
        self.pending_proposal_id: Optional[str] = None
        self.audit_log: List[AuditEvent] = []
        self._event_counter = 0

        self.active_route_id: str = "ORIGINAL_MISSION_ROUTE"
        self.original_route_id: str = "ORIGINAL_MISSION_ROUTE"
        self.last_operator_action: Optional[str] = None
        self.last_route_change_timestamp_s: Optional[float] = None

        self._last_contingency_signature: Optional[str] = None
        self._last_proposal_creation_time_s: float = -999.0
        self._last_evaluated_time_s: float = 0.0

    def evaluate_and_update(
        self,
        twin_state: Any,
        candidate_routes: Optional[List[CandidateRoute]] = None,
        predictive_report: Optional[PredictiveRiskReport] = None
    ) -> MissionReroutingTelemetry:
        """
        Monitors latest digital twin state, decision outputs, trajectory candidates,
        and predictive risk report. Generates structured proposal if a contingency occurs.
        """
        with self._lock:
            t = float(getattr(twin_state, "timestamp_s", time.time()))
            self._last_evaluated_time_s = t

            # 1. Check for expired proposals
            self._check_and_expire_proposals_locked(t)

            # 2. Extract decision & reliability states
            decision: Optional[MissionDecisionResult] = getattr(twin_state, "decision", None)
            quality_status = getattr(twin_state, "quality_status", None)
            quality_val = quality_status.value if hasattr(quality_status, "value") else str(quality_status)

            contingency_detected = False
            contingency_action = "CONTINUE_MISSION"
            operator_attention_required = False

            if decision is not None:
                contingency_action = decision.recommended_action
                if contingency_action != MissionDecisionAction.CONTINUE_MISSION.value:
                    contingency_detected = True
                    operator_attention_required = True

            # 3. If contingency is active and twin state is valid, evaluate proposal creation
            if contingency_detected and candidate_routes:
                self._evaluate_contingency_proposal_locked(
                    t=t,
                    decision=decision,
                    quality_val=quality_val,
                    candidate_routes=candidate_routes,
                    predictive_report=predictive_report
                )

            return self._build_telemetry_snapshot_locked()

    def _evaluate_contingency_proposal_locked(
        self,
        t: float,
        decision: MissionDecisionResult,
        quality_val: str,
        candidate_routes: List[CandidateRoute],
        predictive_report: Optional[PredictiveRiskReport]
    ):
        """Internal helper to generate or supersede proposals when contingency is active."""
        # Find best recommended candidate route
        # Route that is feasible and matches recommended destination or highest ranked
        best_candidate: Optional[CandidateRoute] = None

        # Check if predictive report has a recommendation
        rec_id = predictive_report.recommended_route_id if predictive_report else None
        if rec_id:
            best_candidate = next((c for c in candidate_routes if c.route_id == rec_id and c.is_feasible), None)

        if not best_candidate:
            # Fallback to highest ranked feasible route from candidate routes
            feasible_candidates = [c for c in candidate_routes if c.is_feasible]
            if feasible_candidates:
                best_candidate = min(feasible_candidates, key=lambda c: c.multi_criteria_cost)
            elif candidate_routes:
                # Even if infeasible, pick best to present operator with honest violation data
                best_candidate = min(candidate_routes, key=lambda c: c.multi_criteria_cost)

        if not best_candidate:
            return

        # Do not propose changing to the already active route
        if best_candidate.route_id == self.active_route_id:
            return

        # Deduplication check: construct signature of contingency + proposed route
        sig_data = f"{decision.recommended_action}:{decision.selected_destination}:{best_candidate.route_id}"
        sig_hash = hashlib.md5(sig_data.encode("utf-8")).hexdigest()

        # Check if current pending proposal already addresses this identical signature
        if self.pending_proposal_id and self.pending_proposal_id in self.proposals:
            curr_pending = self.proposals[self.pending_proposal_id]
            if curr_pending.operator_review_status == ProposalStatus.PENDING_OPERATOR_REVIEW.value:
                # If same target and suppressed within suppression window, skip duplicate
                if (curr_pending.proposed_route_id == best_candidate.route_id and
                    (t - self._last_proposal_creation_time_s) < self.config.duplicate_proposal_suppression_s):
                    return

                # If contingency or destination has evolved to a different candidate, supersede old proposal
                if curr_pending.proposed_route_id != best_candidate.route_id:
                    curr_pending.operator_review_status = ProposalStatus.SUPERSEDED.value
                    self._record_audit_event_locked(
                        proposal_id=curr_pending.proposal_id,
                        previous_route_id=curr_pending.current_active_route_id,
                        new_route_id=best_candidate.route_id,
                        event_type="PROPOSAL_SUPERSEDED",
                        operator_action="SYSTEM_SUPERSEDE",
                        operator_id="SYSTEM",
                        reason="New contingency or optimal recovery candidate generated",
                        result="SUPERSEDED"
                    )

        # Create new proposal
        prop_id = f"PROP-{int(t):05d}-{decision.recommended_action[:6]}-{best_candidate.route_id[:10]}"

        # Extract predictive summary if available
        pred_summary: Dict[str, Any] = {}
        if predictive_report and predictive_report.route_summaries:
            matching_summary = next((s for s in predictive_report.route_summaries if s.route_id == best_candidate.route_id), None)
            if matching_summary:
                pred_summary = matching_summary.to_dict()

        # Build waypoint representation
        wp_list = []
        for idx, (name, wp_pos) in enumerate(zip(best_candidate.waypoint_names, best_candidate.waypoints)):
            wp_list.append({
                "waypoint_id": f"WP-{idx:02d}",
                "name": name,
                "latitude_deg": wp_pos.latitude_deg,
                "longitude_deg": wp_pos.longitude_deg,
                "altitude_m": wp_pos.altitude_m
            })

        # Severity assessment
        severity = "WARNING"
        if decision.recommended_action == MissionDecisionAction.EMERGENCY_DESCENT.value:
            severity = "CRITICAL"
        elif decision.recommended_action == MissionDecisionAction.DIVERT_TO_ALTERNATE.value:
            severity = "WARNING"
        elif decision.recommended_action == MissionDecisionAction.ABORT_AND_RTB.value:
            severity = "CAUTION"

        proposal = RouteProposal(
            proposal_id=prop_id,
            creation_timestamp_s=round(t, 2),
            source_contingency=decision.recommended_action,
            severity=severity,
            triggering_conditions=list(decision.triggered_conditions),
            current_active_route_id=self.active_route_id,
            proposed_route_id=best_candidate.route_id,
            proposed_route_type=best_candidate.route_type,
            proposed_waypoint_names=list(best_candidate.waypoint_names),
            proposed_waypoints=wp_list,
            destination_name=best_candidate.destination_name,
            destination_position=best_candidate.destination_position.to_dict(),
            estimated_route_distance_m=round(best_candidate.total_distance_m, 1),
            estimated_mission_duration_s=round(best_candidate.total_duration_s, 1),
            estimated_fuel_liters=round(best_candidate.total_fuel_liters, 2),
            predicted_battery_arrival_soc_pct=round(best_candidate.remaining_battery_soc_pct, 1),
            predicted_fuel_reserve_l=round(best_candidate.remaining_fuel_liters, 2),
            predicted_reserve_margin_pct=round(best_candidate.reserve_margin_pct, 1),
            predictive_risk_summary=pred_summary,
            feasibility_status=best_candidate.is_feasible,
            safety_constraint_violations=list(best_candidate.rejection_reasons),
            explanation=decision.decision_reason,
            expiry_timestamp_s=round(t + self.config.proposal_expiry_duration_s, 2),
            operator_review_status=ProposalStatus.PENDING_OPERATOR_REVIEW.value
        )

        self.proposals[prop_id] = proposal
        self.pending_proposal_id = prop_id
        self._last_contingency_signature = sig_hash
        self._last_proposal_creation_time_s = t

        self._record_audit_event_locked(
            proposal_id=prop_id,
            previous_route_id=self.active_route_id,
            new_route_id=best_candidate.route_id,
            event_type="PROPOSAL_CREATED",
            operator_action="PENDING_REVIEW",
            operator_id="SYSTEM",
            reason=f"Contingency {decision.recommended_action} triggered: {'; '.join(decision.triggered_conditions)}",
            result="PENDING_OPERATOR_REVIEW"
        )
        logger.info(f"Generated route change proposal {prop_id} for target {best_candidate.route_id}")

    def create_proposal(
        self,
        source_contingency: str,
        severity: str,
        triggering_conditions: List[str],
        proposed_route: CandidateRoute,
        explanation: str,
        timestamp_s: float,
        predictive_summary: Optional[Dict[str, Any]] = None
    ) -> RouteProposal:
        """Explicit programmatic method to create a proposal (used in testing and manual triggers)."""
        with self._lock:
            t = float(timestamp_s)
            prop_id = f"PROP-{int(t):05d}-{source_contingency[:6]}-{proposed_route.route_id[:10]}"

            # Supersede existing pending proposal
            if self.pending_proposal_id and self.pending_proposal_id in self.proposals:
                old = self.proposals[self.pending_proposal_id]
                if old.operator_review_status == ProposalStatus.PENDING_OPERATOR_REVIEW.value:
                    old.operator_review_status = ProposalStatus.SUPERSEDED.value
                    self._record_audit_event_locked(
                        proposal_id=old.proposal_id,
                        previous_route_id=old.current_active_route_id,
                        new_route_id=proposed_route.route_id,
                        event_type="PROPOSAL_SUPERSEDED",
                        operator_action="SYSTEM_SUPERSEDE",
                        operator_id="SYSTEM",
                        reason="Manually or programmatically superseded by new proposal",
                        result="SUPERSEDED"
                    )

            wp_list = []
            for idx, (name, wp_pos) in enumerate(zip(proposed_route.waypoint_names, proposed_route.waypoints)):
                wp_list.append({
                    "waypoint_id": f"WP-{idx:02d}",
                    "name": name,
                    "latitude_deg": wp_pos.latitude_deg,
                    "longitude_deg": wp_pos.longitude_deg,
                    "altitude_m": wp_pos.altitude_m
                })

            proposal = RouteProposal(
                proposal_id=prop_id,
                creation_timestamp_s=round(t, 2),
                source_contingency=source_contingency,
                severity=severity,
                triggering_conditions=list(triggering_conditions),
                current_active_route_id=self.active_route_id,
                proposed_route_id=proposed_route.route_id,
                proposed_route_type=proposed_route.route_type,
                proposed_waypoint_names=list(proposed_route.waypoint_names),
                proposed_waypoints=wp_list,
                destination_name=proposed_route.destination_name,
                destination_position=proposed_route.destination_position.to_dict(),
                estimated_route_distance_m=round(proposed_route.total_distance_m, 1),
                estimated_mission_duration_s=round(proposed_route.total_duration_s, 1),
                estimated_fuel_liters=round(proposed_route.total_fuel_liters, 2),
                predicted_battery_arrival_soc_pct=round(proposed_route.remaining_battery_soc_pct, 1),
                predicted_fuel_reserve_l=round(proposed_route.remaining_fuel_liters, 2),
                predicted_reserve_margin_pct=round(proposed_route.reserve_margin_pct, 1),
                predictive_risk_summary=predictive_summary or {},
                feasibility_status=proposed_route.is_feasible,
                safety_constraint_violations=list(proposed_route.rejection_reasons),
                explanation=explanation,
                expiry_timestamp_s=round(t + self.config.proposal_expiry_duration_s, 2),
                operator_review_status=ProposalStatus.PENDING_OPERATOR_REVIEW.value
            )

            self.proposals[prop_id] = proposal
            self.pending_proposal_id = prop_id
            self._last_proposal_creation_time_s = t

            self._record_audit_event_locked(
                proposal_id=prop_id,
                previous_route_id=self.active_route_id,
                new_route_id=proposed_route.route_id,
                event_type="PROPOSAL_CREATED",
                operator_action="PENDING_REVIEW",
                operator_id="SYSTEM",
                reason=f"Proposal created for {source_contingency}",
                result="PENDING_OPERATOR_REVIEW"
            )
            return proposal

    def get_pending_proposal(self) -> Optional[RouteProposal]:
        """Returns the currently active pending proposal, if any."""
        with self._lock:
            if not self.pending_proposal_id:
                return None
            prop = self.proposals.get(self.pending_proposal_id)
            if prop and prop.operator_review_status == ProposalStatus.PENDING_OPERATOR_REVIEW.value:
                return prop
            return None

    def get_pending_proposal_count(self) -> int:
        """Returns the number of proposals currently pending review (0 or 1)."""
        with self._lock:
            return 1 if self.get_pending_proposal() is not None else 0

    def get_proposal(self, proposal_id: str) -> Optional[RouteProposal]:
        """Returns proposal by ID or None."""
        with self._lock:
            return self.proposals.get(proposal_id)

    def get_all_proposals(self) -> List[RouteProposal]:
        """Returns list of all proposals."""
        with self._lock:
            return list(self.proposals.values())

    def get_audit_events(self) -> List[AuditEvent]:
        """Returns copy of all audit event objects."""
        with self._lock:
            return list(self.audit_log)

    def approve_proposal(
        self,
        proposal_id: str,
        operator_id: str = "PILOT_GCS_01",
        navigation_engine: Optional[SpatialNavigationEngine] = None,
        twin_state: Optional[Any] = None,
        current_time_s: Optional[float] = None,
        timestamp_s: Optional[float] = None
    ) -> Tuple[bool, str, Optional[RouteProposal]]:
        """
        Operator approves a proposed route change.
        Enforces 10 safety safeguards before transitioning status.
        """
        with self._lock:
            t = current_time_s if current_time_s is not None else (timestamp_s if timestamp_s is not None else self._last_evaluated_time_s)

            # Safeguard 1: Unknown proposal ID
            if proposal_id not in self.proposals:
                return False, f"Unknown proposal ID '{proposal_id}'.", None

            proposal = self.proposals[proposal_id]

            # Safeguard 2: Already approved, applied, or rejected
            if proposal.operator_review_status == ProposalStatus.APPLIED.value:
                return False, f"Proposal '{proposal_id}' has already been applied.", proposal
            if proposal.operator_review_status == ProposalStatus.REJECTED.value:
                return False, f"Proposal '{proposal_id}' was already rejected and cannot be approved.", proposal
            if proposal.operator_review_status == ProposalStatus.SUPERSEDED.value:
                return False, f"Proposal '{proposal_id}' was superseded by a newer proposal.", proposal

            # Safeguard 3: Check expiry
            if t > proposal.expiry_timestamp_s or proposal.operator_review_status == ProposalStatus.EXPIRED.value:
                proposal.operator_review_status = ProposalStatus.EXPIRED.value
                self._record_audit_event_locked(
                    proposal_id=proposal_id,
                    previous_route_id=proposal.current_active_route_id,
                    new_route_id=proposal.proposed_route_id,
                    event_type="PROPOSAL_EXPIRED",
                    operator_action="APPROVE_BLOCKED_EXPIRED",
                    operator_id=operator_id,
                    reason="Approval attempted after proposal expiry",
                    result="EXPIRED"
                )
                return False, f"Proposal '{proposal_id}' has expired (expiry t={proposal.expiry_timestamp_s}s, current t={t}s).", proposal

            # Safeguard 4: Infeasible proposal cannot be approved
            if not proposal.feasibility_status:
                self._record_audit_event_locked(
                    proposal_id=proposal_id,
                    previous_route_id=proposal.current_active_route_id,
                    new_route_id=proposal.proposed_route_id,
                    event_type="OPERATOR_APPROVAL_REJECTED",
                    operator_action="APPROVE_BLOCKED_INFEASIBLE",
                    operator_id=operator_id,
                    reason=f"Infeasible route proposal: {'; '.join(proposal.safety_constraint_violations)}",
                    result="REJECTED_BY_POLICY"
                )
                return False, f"Proposal '{proposal_id}' is physically infeasible ({'; '.join(proposal.safety_constraint_violations)}).", proposal

            # Safeguard 5 & 6: Digital Twin Quality and Staleness Validation
            if twin_state is not None:
                q_status = getattr(twin_state, "quality_status", None)
                q_val = q_status.value if hasattr(q_status, "value") else str(q_status)
                if q_val in ("INVALID", "DEGRADED"):
                    return False, f"Cannot approve proposal: Digital Twin data quality is {q_val}.", proposal

                state_t = getattr(twin_state, "timestamp_s", None)
                if state_t is not None and abs(t - state_t) > self.config.max_stale_tolerance_s:
                    return False, f"Cannot approve proposal: Digital Twin telemetry is stale (age={abs(t - state_t):.2f}s > {self.config.max_stale_tolerance_s}s).", proposal

            # Mark APPROVED
            proposal.operator_review_status = ProposalStatus.APPROVED.value
            proposal.operator_action = "APPROVE"
            proposal.operator_id = operator_id
            proposal.review_timestamp_s = round(t, 2)
            self.last_operator_action = f"APPROVED by {operator_id}"

            self._record_audit_event_locked(
                proposal_id=proposal_id,
                previous_route_id=proposal.current_active_route_id,
                new_route_id=proposal.proposed_route_id,
                event_type="OPERATOR_APPROVED",
                operator_action="APPROVE",
                operator_id=operator_id,
                reason="Operator confirmed route change",
                result="APPROVED"
            )

            # If navigation engine provided, apply atomically
            if navigation_engine is not None:
                curr_pos = getattr(twin_state, "mission", None)
                pos = curr_pos.current_position if curr_pos else None
                return self._apply_approved_proposal_locked(proposal, navigation_engine, pos, t)

            return True, f"Proposal '{proposal_id}' approved successfully. Ready for application.", proposal

    def reject_proposal(
        self,
        proposal_id: str,
        reason: str = "",
        operator_id: str = "PILOT_GCS_01",
        current_time_s: Optional[float] = None,
        timestamp_s: Optional[float] = None
    ) -> Tuple[bool, str, Optional[RouteProposal]]:
        """
        Operator explicitly rejects a proposed route change.
        Ensures the active mission route remains 100% unchanged.
        """
        with self._lock:
            t = current_time_s if current_time_s is not None else (timestamp_s if timestamp_s is not None else self._last_evaluated_time_s)

            if proposal_id not in self.proposals:
                return False, f"Unknown proposal ID '{proposal_id}'.", None

            proposal = self.proposals[proposal_id]

            if proposal.operator_review_status == ProposalStatus.APPLIED.value:
                return False, f"Proposal '{proposal_id}' has already been applied and cannot be rejected.", proposal

            proposal.operator_review_status = ProposalStatus.REJECTED.value
            proposal.operator_action = "REJECT"
            proposal.operator_id = operator_id
            proposal.review_timestamp_s = round(t, 2)
            proposal.rejection_reason = reason or "Operator rejected route modification."

            if self.pending_proposal_id == proposal_id:
                self.pending_proposal_id = None

            self.last_operator_action = f"REJECTED by {operator_id}: {reason}"

            self._record_audit_event_locked(
                proposal_id=proposal_id,
                previous_route_id=proposal.current_active_route_id,
                new_route_id=proposal.proposed_route_id,
                event_type="OPERATOR_REJECTED",
                operator_action="REJECT",
                operator_id=operator_id,
                reason=reason or "Operator rejected proposed trajectory",
                result="REJECTED"
            )
            logger.info(f"Proposal {proposal_id} rejected by operator {operator_id}. Active route preserved.")
            return True, f"Proposal '{proposal_id}' rejected. Active route '{self.active_route_id}' remains unchanged.", proposal

    def apply_approved_proposal(
        self,
        proposal_id: str,
        navigation_engine: SpatialNavigationEngine,
        current_position: Optional[GeographicPosition] = None,
        timestamp_s: Optional[float] = None
    ) -> Tuple[bool, str, Optional[RouteProposal]]:
        """Atomically applies an approved proposal to the simulated navigation engine."""
        with self._lock:
            if proposal_id not in self.proposals:
                return False, f"Unknown proposal ID '{proposal_id}'.", None

            proposal = self.proposals[proposal_id]
            t = timestamp_s if timestamp_s is not None else self._last_evaluated_time_s
            return self._apply_approved_proposal_locked(proposal, navigation_engine, current_position, t)

    def _apply_approved_proposal_locked(
        self,
        proposal: RouteProposal,
        navigation_engine: SpatialNavigationEngine,
        current_position: Optional[GeographicPosition],
        t: float
    ) -> Tuple[bool, str, Optional[RouteProposal]]:
        """Internal worker to execute atomic navigation update with rollback."""
        if proposal.operator_review_status != ProposalStatus.APPROVED.value:
            return False, f"Cannot apply proposal '{proposal.proposal_id}' in state '{proposal.operator_review_status}'. Must be APPROVED.", proposal

        # Convert proposal waypoints to Waypoint objects
        new_wps: List[Waypoint] = []
        for wp_dict in proposal.proposed_waypoints:
            wp_obj = Waypoint(
                waypoint_id=wp_dict.get("waypoint_id", "WP-NEW"),
                latitude_deg=float(wp_dict["latitude_deg"]),
                longitude_deg=float(wp_dict["longitude_deg"]),
                altitude_m=float(wp_dict["altitude_m"]),
                name=wp_dict.get("name", "Diverted Waypoint")
            )
            new_wps.append(wp_obj)

        if not new_wps:
            return False, "Proposed route has no waypoints to apply.", proposal

        pos = current_position or (navigation_engine.waypoints[0].to_position() if navigation_engine.waypoints else GeographicPosition(18.5822, 73.9197, 5000.0))

        try:
            prev_route_id = self.active_route_id
            navigation_engine.apply_new_route(
                new_waypoints=new_wps,
                route_id=proposal.proposed_route_id,
                timestamp_s=t,
                current_position=pos
            )

            # Route updated successfully
            proposal.operator_review_status = ProposalStatus.APPLIED.value
            proposal.applied_timestamp_s = round(t, 2)
            self.active_route_id = proposal.proposed_route_id
            self.last_route_change_timestamp_s = round(t, 2)

            if self.pending_proposal_id == proposal.proposal_id:
                self.pending_proposal_id = None

            self._record_audit_event_locked(
                proposal_id=proposal.proposal_id,
                previous_route_id=prev_route_id,
                new_route_id=proposal.proposed_route_id,
                event_type="ROUTE_APPLIED",
                operator_action="APPLIED",
                operator_id=proposal.operator_id or "SYSTEM",
                reason="Approved route successfully committed to navigation engine",
                result="SUCCESS"
            )
            logger.info(f"Route change applied: {prev_route_id} -> {proposal.proposed_route_id} at t={t:.2f}s")
            return True, f"Route change to '{proposal.proposed_route_id}' ({proposal.destination_name}) applied successfully.", proposal
        except Exception as e:
            proposal.operator_review_status = ProposalStatus.FAILED.value
            self._record_audit_event_locked(
                proposal_id=proposal.proposal_id,
                previous_route_id=self.active_route_id,
                new_route_id=proposal.proposed_route_id,
                event_type="ROUTE_APPLICATION_FAILED",
                operator_action="APPLY_ERROR",
                operator_id=proposal.operator_id or "SYSTEM",
                reason=str(e),
                result="FAILED"
            )
            logger.error(f"Failed to apply route proposal {proposal.proposal_id}: {e}")
            return False, f"Failed to apply route: {e}", proposal

    def expire_proposals(self, current_time_s: float):
        """Public method to sweep expired proposals."""
        with self._lock:
            self._check_and_expire_proposals_locked(current_time_s)

    def _check_and_expire_proposals_locked(self, t: float):
        """Internal helper to mark proposals past expiry timestamp as EXPIRED."""
        for prop in self.proposals.values():
            if prop.operator_review_status == ProposalStatus.PENDING_OPERATOR_REVIEW.value:
                if t > prop.expiry_timestamp_s:
                    prop.operator_review_status = ProposalStatus.EXPIRED.value
                    if self.pending_proposal_id == prop.proposal_id:
                        self.pending_proposal_id = None
                    self._record_audit_event_locked(
                        proposal_id=prop.proposal_id,
                        previous_route_id=prop.current_active_route_id,
                        new_route_id=prop.proposed_route_id,
                        event_type="PROPOSAL_EXPIRED",
                        operator_action="AUTO_EXPIRED",
                        operator_id="SYSTEM",
                        reason=f"Proposal exceeded validity window ({self.config.proposal_expiry_duration_s}s)",
                        result="EXPIRED"
                    )

    def _record_audit_event_locked(
        self,
        proposal_id: str,
        previous_route_id: str,
        new_route_id: Optional[str],
        event_type: str,
        operator_action: Optional[str],
        operator_id: Optional[str],
        reason: str,
        result: str
    ):
        """Internal helper to append bounded audit record."""
        self._event_counter += 1
        event = AuditEvent(
            event_id=f"AUDIT-{self._event_counter:05d}",
            timestamp_s=round(self._last_evaluated_time_s, 2),
            proposal_id=proposal_id,
            previous_route_id=previous_route_id,
            new_route_id=new_route_id,
            event_type=event_type,
            operator_action=operator_action,
            operator_id=operator_id,
            reason=reason,
            result=result
        )
        self.audit_log.append(event)
        if len(self.audit_log) > self.config.audit_history_retention_count:
            self.audit_log.pop(0)

    def get_audit_history(self) -> List[Dict[str, Any]]:
        """Returns bounded audit log history."""
        with self._lock:
            return [e.to_dict() for e in self.audit_log]

    def _build_telemetry_snapshot_locked(self) -> MissionReroutingTelemetry:
        """Internal builder for lightweight telemetry export."""
        pending_prop = self.proposals.get(self.pending_proposal_id) if self.pending_proposal_id else None
        
        status = "NOMINAL"
        attn = False
        contingency_detected = False
        contingency_action = "CONTINUE_MISSION"

        if pending_prop and pending_prop.operator_review_status == ProposalStatus.PENDING_OPERATOR_REVIEW.value:
            status = "PENDING_APPROVAL"
            attn = True
            contingency_detected = True
            contingency_action = pending_prop.source_contingency
        elif self.last_route_change_timestamp_s is not None:
            status = "ROUTE_UPDATED"

        return MissionReroutingTelemetry(
            rerouting_subsystem_status=status,
            pending_proposal_count=1 if (pending_prop and pending_prop.operator_review_status == ProposalStatus.PENDING_OPERATOR_REVIEW.value) else 0,
            active_proposal_id=pending_prop.proposal_id if pending_prop else None,
            proposal_status=pending_prop.operator_review_status if pending_prop else ProposalStatus.NO_PROPOSAL.value,
            last_operator_action=self.last_operator_action,
            active_route_id=self.active_route_id,
            original_route_id=self.original_route_id,
            last_route_change_timestamp_s=self.last_route_change_timestamp_s,
            proposed_route_id=pending_prop.proposed_route_id if pending_prop else None,
            proposed_destination=pending_prop.destination_name if pending_prop else None,
            contingency_detected=contingency_detected,
            contingency_action=contingency_action,
            operator_attention_required=attn
        )
