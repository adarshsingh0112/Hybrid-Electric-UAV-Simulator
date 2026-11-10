"""
ENGINE-TWIN: Real-Time Digital Twin Streaming Backend Service (SIH26054)
FastAPI + WebSockets service running:
- 20 Hz continuous MVEM physics integration
- Datasheet sensor simulation with noise & dynamics
- Extended Kalman Filter (EKF) state estimation
- Normalized physics residual calculation
- AI Anomaly Detection + Multi-Task Fault Classification + RUL Prognostics
- Fast SHAP feature attribution + Operator Alert Generation
- High-frequency WebSocket broadcast to interactive 3D dashboard
"""
import os
import sys
import asyncio
import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Any
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse, StreamingResponse
from pydantic import BaseModel

# Ensure project root is in path
PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from simulation.flight_profile import FlightProfile, FlightState
from simulation.mvem import MeanValueEngineModel, EngineState
from simulation.sensors import SensorModel, SensorReadings
from simulation.fault_injector import FaultInjector, FaultType, FaultConfig
from simulation.hybrid_powertrain import HybridPowertrain, HybridOperatingMode
from digital_twin.uav_digital_twin import UAVDigitalTwin
from digital_twin.health_index import HealthIndexEngine
from digital_twin.ekf_estimator import ExtendedKalmanFilter
from digital_twin.twin_state import DigitalTwinState, SubsystemHealth, AIHealthState, StateLevel
from models.sensor_validator import SensorValidator
from models.anomaly_autoencoder import AnomalyDetector
from models.fault_classifier import FaultDiagnosisEngine
from models.rul_estimator import RULEstimator
from xai.shap_explainer import FastSHAPExplainer
from xai.alert_generator import AlertGenerator

app = FastAPI(title="ENGINE-TWIN Real-Time Digital Twin Server")

# Serve dashboard static assets
DASHBOARD_DIR = PROJECT_ROOT / "dashboard"
DASHBOARD_DIR.mkdir(parents=True, exist_ok=True)
app.mount("/static", StaticFiles(directory=str(DASHBOARD_DIR)), name="static")

# Global Engine Twin Runtime State
class EngineTwinRuntime:
    def __init__(self):
        self.flight_gen = FlightProfile()
        self.mvem_physical = MeanValueEngineModel()      # Physical/Simulated engine (subject to faults)
        self.mvem_baseline = MeanValueEngineModel()      # Healthy Digital Twin baseline
        self.sensors = SensorModel(seed=42)
        self.injector = FaultInjector(self.mvem_physical, self.sensors)
        self.ekf = ExtendedKalmanFilter()
        self.health_engine = HealthIndexEngine()
        self.sensor_validator = SensorValidator()
        self.rul_engine = RULEstimator()

        self.hybrid_powertrain = HybridPowertrain(engine=self.mvem_physical)
        self.uav_digital_twin = UAVDigitalTwin()
        self.mode_override: Optional[HybridOperatingMode] = None

        # Load AI Models
        models_dir = PROJECT_ROOT / "models" / "saved_models"
        ae_path = models_dir / "anomaly_autoencoder.pt"
        clf_path = models_dir / "fault_classifier.pt"

        self.anomaly_detector = AnomalyDetector(str(ae_path) if ae_path.exists() else None)
        self.fault_classifier = FaultDiagnosisEngine(str(clf_path) if clf_path.exists() else None)
        self.shap_explainer = FastSHAPExplainer(self.fault_classifier.model if self.fault_classifier.is_trained else None)

        self.sim_time_s = 0.0
        self.time_scale = 1.0
        self.is_running = True
        self.total_mission_duration_s = 3600.0

        # Latest State Snapshot
        self.latest_state: Optional[DigitalTwinState] = None
        self.latest_alert: Optional[Dict[str, Any]] = None
        self.active_fault_name: str = "HEALTHY"

    def reset(self):
        self.sim_time_s = 0.0
        self.mvem_physical.reset(idle=False)
        self.mvem_baseline.reset(idle=False)
        self.sensors = SensorModel(seed=int(time.time()))
        self.injector = FaultInjector(self.mvem_physical, self.sensors)
        self.ekf = ExtendedKalmanFilter()
        self.rul_engine.reset()
        self.hybrid_powertrain.reset()
        self.uav_digital_twin.reset()
        self.mode_override = None
        self.latest_alert = None
        self.active_fault_name = "HEALTHY"

    def step(self, dt_s: float = 0.05) -> DigitalTwinState:
        self.sim_time_s += dt_s * self.time_scale
        t = self.sim_time_s

        # 1. Flight State
        flight = self.flight_gen.get_standard_mission_state(t, self.total_mission_duration_s)

        # 2. Update Fault Injection Progress
        self.injector.update(t)

        # 3. Simulate Physical Engine & Sensor Measurement
        true_physical = self.mvem_physical.step(flight, dt_s=dt_s)
        sensor_meas = self.sensors.sample(true_physical, flight, dt_s=dt_s)

        # 4. Simulate Healthy Digital Twin Physics Baseline
        mvem_expected = self.mvem_baseline.step(flight, dt_s=dt_s)

        # 5. Extended Kalman Filter Estimation
        self.ekf.predict(mvem_expected, dt_s=dt_s)
        self.ekf.update(sensor_meas)
        ekf_est = self.ekf.get_estimated_state()

        # 6. Normalized Residual Calculation: (y_sensor - y_mvem) / sigma
        sensor_dict = {
            "rpm": sensor_meas.rpm,
            "manifold_pressure": sensor_meas.manifold_pressure_bar,
            "oil_pressure": sensor_meas.oil_pressure_bar,
            "oil_temp": sensor_meas.oil_temp_c,
            "fuel_flow": sensor_meas.fuel_flow_lph,
            "egt_c": sensor_meas.egt_c,
            "cht_c": sensor_meas.cht_c,
            "vibration_rms": sensor_meas.vibration_rms_g,
            "bus_voltage": sensor_meas.bus_voltage_v,
            "ambient_temp_c": sensor_meas.ambient_temp_c
        }
        mvem_dict = {
            "rpm": mvem_expected.rpm,
            "manifold_pressure": mvem_expected.manifold_pressure_bar,
            "oil_pressure": mvem_expected.oil_pressure_bar,
            "oil_temp": mvem_expected.oil_temp_c,
            "fuel_flow": mvem_expected.fuel_flow_lph,
            "egt_c": mvem_expected.egt_c,
            "cht_c": mvem_expected.cht_c,
            "vibration_rms": mvem_expected.vibration_rms_g,
            "bus_voltage": mvem_expected.bus_voltage_v
        }
        residuals = self.health_engine.compute_residuals(sensor_dict, mvem_dict)
        sensor_check = self.sensor_validator.validate(sensor_dict, mvem_dict, residuals)

        if sensor_check.is_sensor_fault:
            # Decouple faulty sensor probe from mechanical health evaluation
            mech_residuals = dict(residuals)
            if sensor_check.faulty_channel:
                mech_residuals[sensor_check.faulty_channel] = 0.0
            subsystem_health = self.health_engine.evaluate_health(mech_residuals)
            subsystem_health.status_level = StateLevel.ADVISORY
        else:
            subsystem_health = self.health_engine.evaluate_health(residuals)

        # 9. AI Anomaly Detection & Fault Diagnosis
        is_anom, raw_mse, anom_thresh, anom_index = self.anomaly_detector.detect(residuals)
        
        if sensor_check.is_sensor_fault:
            fault_class = "SENSOR_FAULT_EGT3" if "egt" in (sensor_check.faulty_channel or "") else "SENSOR_FAULT_GENERAL"
            conf = sensor_check.confidence_pct
            sev = 0.25
        elif is_anom or subsystem_health.overall_health < 85.0:
            fault_class, conf, sev, _ = self.fault_classifier.diagnose(residuals)
        else:
            fault_class = "HEALTHY"
            conf = 99.8
            sev = 0.0

        # 10. RUL Prognostics
        rul_pred = self.rul_engine.update(t, subsystem_health.overall_health)

        # 11. XAI SHAP Attribution
        shap_exps = self.shap_explainer.explain(residuals, fault_class)

        # 12. Operator Alert Generation
        alert = AlertGenerator.generate(
            timestamp_s=t,
            fault_class=fault_class,
            confidence_pct=conf,
            severity=sev,
            is_sensor_fault=sensor_check.is_sensor_fault,
            sensor_check_explanation=sensor_check.explanation,
            shap_explanations=shap_exps,
            rul_prediction=rul_pred,
            overall_health=subsystem_health.overall_health
        )
        self.latest_alert = alert.to_dict() if alert else None

        ai_health_state = AIHealthState(
            anomaly_detected=is_anom,
            anomaly_score=anom_index,
            anomaly_raw_mse=raw_mse,
            anomaly_threshold=anom_thresh,
            fault_class=fault_class,
            fault_confidence_pct=conf,
            fault_severity=sev,
            is_sensor_fault=sensor_check.is_sensor_fault,
            rul_hours_mean=rul_pred.rul_hours_mean,
            rul_hours_min=rul_pred.rul_hours_min,
            rul_hours_max=rul_pred.rul_hours_max,
            top_contributing_channels=shap_exps,
            recommended_action=alert.recommended_action if alert else "Continue nominal mission profile."
        )

        # 13. Step Hybrid Powertrain & Unified UAV Digital Twin
        prop_req_w = max(5000.0, (flight.throttle_pct / 100.0) * 85000.0)
        hybrid_telemetry = self.hybrid_powertrain.update(
            propulsion_power_request_w=prop_req_w,
            flight_state=flight,
            ambient_temp_c=flight.ambient_temp_c,
            dt_s=dt_s,
            mode_override=self.mode_override
        )
        uav_snapshot = self.uav_digital_twin.update_from_hybrid_telemetry(
            hybrid_telemetry=hybrid_telemetry,
            flight_state=flight,
            engine_health_pct=subsystem_health.overall_health,
            dt_s=dt_s
        )

        state = DigitalTwinState(
            timestamp_s=round(t, 2),
            altitude_ft=round(flight.altitude_ft, 1),
            ambient_temp_c=round(flight.ambient_temp_c, 2),
            throttle_pct=round(flight.throttle_pct, 1),
            airspeed_mps=round(flight.airspeed_mps, 1),
            mission_phase=flight.phase,
            # Sensor
            sensor_rpm=round(sensor_meas.rpm, 1),
            sensor_map_bar=round(sensor_meas.manifold_pressure_bar, 3),
            sensor_oil_p_bar=round(sensor_meas.oil_pressure_bar, 2),
            sensor_oil_t_c=round(sensor_meas.oil_temp_c, 1),
            sensor_coolant_t_c=round(sensor_meas.coolant_temp_c, 1),
            sensor_fuel_flow_lph=round(sensor_meas.fuel_flow_lph, 2),
            sensor_egt_c=[round(x, 1) for x in sensor_meas.egt_c],
            sensor_cht_c=[round(x, 1) for x in sensor_meas.cht_c],
            sensor_vib_rms_g=round(sensor_meas.vibration_rms_g, 3),
            sensor_bus_v=round(sensor_meas.bus_voltage_v, 2),
            # MVEM Expected
            mvem_expected_power_hp=round(mvem_expected.power_hp, 1),
            mvem_expected_rpm=round(mvem_expected.rpm, 1),
            mvem_expected_map_bar=round(mvem_expected.manifold_pressure_bar, 3),
            mvem_expected_oil_p_bar=round(mvem_expected.oil_pressure_bar, 2),
            mvem_expected_oil_t_c=round(mvem_expected.oil_temp_c, 1),
            mvem_expected_egt_c=[round(x, 1) for x in mvem_expected.egt_c],
            mvem_expected_cht_c=[round(x, 1) for x in mvem_expected.cht_c],
            mvem_expected_fuel_lph=round(mvem_expected.fuel_flow_lph, 2),
            mvem_expected_vib_rms_g=round(mvem_expected.vibration_rms_g, 3),
            # Estimated EKF
            estimated_rpm=ekf_est["estimated_rpm"],
            estimated_map_bar=ekf_est["estimated_map_bar"],
            estimated_oil_p_bar=ekf_est["estimated_oil_p_bar"],
            estimated_oil_t_c=ekf_est["estimated_oil_t_c"],
            estimated_egt_c=ekf_est["estimated_egt_c"],
            estimated_cht_c=ekf_est["estimated_cht_c"],
            # Residuals & Health
            residuals=residuals,
            health=subsystem_health,
            ai_prognostics=ai_health_state,
            active_fault=self.active_fault_name,
            uav_twin=uav_snapshot.to_dict(),
            provenance="SIMULATED"
        )
        self.latest_state = state
        return state

twin_runtime = EngineTwinRuntime()

# -------------------------------------------------------------
# REST API Endpoints
# -------------------------------------------------------------
class FaultInjectRequest(BaseModel):
    fault_type: str
    severity: float = 1.0
    ramp_duration_s: float = 8.0

@app.get("/")
async def get_dashboard_root():
    """Serves the primary Digital Twin Operator Dashboard."""
    index_file = DASHBOARD_DIR / "index.html"
    if index_file.exists():
        return FileResponse(index_file)
    return HTMLResponse("<h1>ENGINE-TWIN Backend Running. Dashboard initializing...</h1>")

@app.get("/api/state")
async def get_current_state():
    if not twin_runtime.latest_state:
        twin_runtime.step(dt_s=0.05)
    payload = twin_runtime.latest_state.to_dict()
    payload["active_alert"] = twin_runtime.latest_alert
    payload["active_fault"] = twin_runtime.active_fault_name
    return payload

@app.get("/api/stream")
async def sse_telemetry_stream():
    """Server-Sent Events (SSE) telemetry stream for browser dashboard at 20 Hz."""
    async def event_generator():
        while True:
            if twin_runtime.is_running:
                state = twin_runtime.step(dt_s=0.05)
                payload = state.to_dict()
                payload["active_alert"] = twin_runtime.latest_alert
                payload["active_fault"] = twin_runtime.active_fault_name
                yield f"data: {json.dumps(payload)}\n\n"
            await asyncio.sleep(0.05)
    return StreamingResponse(event_generator(), media_type="text/event-stream")

@app.post("/api/fault/inject")
async def inject_fault_endpoint(req: FaultInjectRequest):
    try:
        f_enum = FaultType(req.fault_type)
    except ValueError:
        raise HTTPException(status_code=400, detail=f"Unknown fault type: {req.fault_type}")

    cfg = FaultConfig(
        fault_type=f_enum,
        start_time_s=twin_runtime.sim_time_s,
        ramp_duration_s=req.ramp_duration_s,
        severity=req.severity
    )
    twin_runtime.injector.inject_fault(cfg)
    twin_runtime.active_fault_name = req.fault_type
    return {
        "status": "SUCCESS",
        "injected_fault": req.fault_type,
        "severity": req.severity,
        "start_time_s": twin_runtime.sim_time_s
    }

@app.post("/api/fault/clear")
async def clear_fault_endpoint():
    twin_runtime.injector.clear_faults()
    twin_runtime.active_fault_name = "HEALTHY"
    return {"status": "SUCCESS", "message": "Engine reset to healthy baseline."}

@app.api_route("/api/sim/start", methods=["GET", "POST"])
async def start_simulation_endpoint():
    twin_runtime.is_running = True
    return {"status": "SUCCESS", "is_running": True}

@app.api_route("/api/sim/pause", methods=["GET", "POST"])
async def pause_simulation_endpoint():
    twin_runtime.is_running = False
    return {"status": "SUCCESS", "is_running": False}

@app.api_route("/api/sim/reset", methods=["GET", "POST"])
async def reset_simulation_endpoint():
    twin_runtime.reset()
    return {"status": "SUCCESS", "message": "Simulation restarted."}

@app.api_route("/api/sim/speed", methods=["GET", "POST"])
async def set_simulation_speed(speed: float = 1.0):
    twin_runtime.time_scale = max(0.2, min(10.0, speed))
    return {"status": "SUCCESS", "time_scale": twin_runtime.time_scale}

@app.api_route("/api/hybrid/mode", methods=["GET", "POST"])
async def set_hybrid_mode_endpoint(mode: str = "AUTO"):
    m = mode.upper()
    if m in ["AUTO", "NONE"]:
        twin_runtime.mode_override = None
    else:
        try:
            twin_runtime.mode_override = HybridOperatingMode(m)
        except ValueError:
            raise HTTPException(status_code=400, detail=f"Invalid hybrid mode: {mode}")
    return {"status": "SUCCESS", "mode": mode}

# -------------------------------------------------------------
# STEP 10: Closed-Loop Mission Re-Routing & Confirmation Endpoints
# -------------------------------------------------------------
class OperatorApprovalRequest(BaseModel):
    operator_id: str = "PILOT-GCS-01"

class OperatorRejectionRequest(BaseModel):
    operator_id: str = "PILOT-GCS-01"
    reason: str = "Rejected by operator"

@app.get("/api/mission/route")
async def get_mission_route():
    """Returns active route geometry, progress, and version."""
    twin = twin_runtime.uav_digital_twin
    nav = twin.navigation_engine
    return {
        "status": "SUCCESS",
        "active_route_id": getattr(nav, "active_route_id", "ORIGINAL_MISSION_ROUTE"),
        "original_route_id": getattr(nav, "original_route_id", "ORIGINAL_MISSION_ROUTE"),
        "destination": getattr(nav.home_position, "name", "Pune Lohegaon (Home Base)") if nav.home_position else "Pune Lohegaon",
        "current_position": nav.current_position.to_dict() if nav.current_position else None,
        "current_waypoint_index": nav.current_waypoint_index,
        "total_waypoints": len(nav.waypoints),
        "waypoints": [wp.to_dict() for wp in nav.waypoints],
        "completed_distance_m": nav.completed_route_distance_m,
        "remaining_distance_m": nav.remaining_route_distance_m,
        "total_distance_m": nav.total_planned_route_distance_m,
        "mission_progress_pct": nav.mission_progress_pct,
        "route_version": getattr(nav, "route_version", 1),
        "route_history": getattr(nav, "route_history", []),
        "provenance": "SIMULATED"
    }

@app.get("/api/mission/proposals")
async def get_mission_proposals():
    """Returns all route proposals generated during this session."""
    rm = twin_runtime.uav_digital_twin.rerouting_manager
    return {
        "status": "SUCCESS",
        "proposals": [p.to_dict() for p in rm.get_all_proposals()],
        "pending_count": rm.get_pending_proposal_count(),
        "provenance": "SIMULATED"
    }

@app.get("/api/mission/proposals/pending")
async def get_pending_proposal():
    """Returns the current active pending proposal requiring pilot review, if any."""
    rm = twin_runtime.uav_digital_twin.rerouting_manager
    prop = rm.get_pending_proposal()
    return {
        "status": "SUCCESS",
        "has_pending": prop is not None,
        "proposal": prop.to_dict() if prop else None,
        "provenance": "SIMULATED"
    }

@app.post("/api/mission/proposals/{proposal_id}/approve")
async def approve_mission_proposal(proposal_id: str, req: Optional[OperatorApprovalRequest] = None):
    """Pilot confirmation endpoint: approves proposal and applies route update safely."""
    op_id = req.operator_id if req else "PILOT-GCS-01"
    twin = twin_runtime.uav_digital_twin
    rm = twin.rerouting_manager
    latest_state = twin.latest_state
    
    if latest_state is None:
        twin_runtime.step(dt_s=0.05)
        latest_state = twin.latest_state
        
    success, message, proposal = rm.approve_proposal(
        proposal_id=proposal_id,
        operator_id=op_id,
        twin_state=latest_state,
        navigation_engine=twin.navigation_engine
    )
    if not success:
        raise HTTPException(status_code=400, detail=message)
        
    return {
        "status": "SUCCESS",
        "message": message,
        "proposal": proposal.to_dict() if proposal else None,
        "active_route_id": twin.navigation_engine.active_route_id,
        "provenance": "SIMULATED"
    }

@app.post("/api/mission/proposals/{proposal_id}/reject")
async def reject_mission_proposal(proposal_id: str, req: Optional[OperatorRejectionRequest] = None):
    """Pilot rejection endpoint: rejects proposal and logs rejection reason."""
    op_id = req.operator_id if req else "PILOT-GCS-01"
    reason = req.reason if req else "Rejected by operator"
    rm = twin_runtime.uav_digital_twin.rerouting_manager
    latest_state = twin_runtime.uav_digital_twin.latest_state
    timestamp_s = latest_state.timestamp_s if latest_state else twin_runtime.sim_time_s
    
    success, message, proposal = rm.reject_proposal(
        proposal_id=proposal_id,
        operator_id=op_id,
        reason=reason,
        timestamp_s=timestamp_s
    )
    if not success:
        raise HTTPException(status_code=400, detail=message)
        
    return {
        "status": "SUCCESS",
        "message": message,
        "proposal": proposal.to_dict() if proposal else None,
        "provenance": "SIMULATED"
    }

@app.get("/api/mission/audit")
async def get_mission_audit_trail():
    """Returns immutable audit event trail."""
    rm = twin_runtime.uav_digital_twin.rerouting_manager
    return {
        "status": "SUCCESS",
        "audit_events": [e.to_dict() for e in rm.get_audit_events()],
        "total_events": len(rm.get_audit_events()),
        "provenance": "SIMULATED"
    }


# -------------------------------------------------------------
# WebSocket Telemetry Stream (20 Hz Broadcast)
# -------------------------------------------------------------
@app.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("Dashboard client connected to telemetry stream.")
    try:
        while True:
            state = twin_runtime.step(dt_s=0.05)
            payload = state.to_dict()
            if twin_runtime.latest_alert:
                payload["active_alert"] = twin_runtime.latest_alert
            else:
                payload["active_alert"] = None

            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(0.05) # 20 Hz update cadence
    except WebSocketDisconnect:
        print("Dashboard client disconnected.")
    except Exception as e:
        print(f"WebSocket streaming error: {e}")

if __name__ == "__main__":
    import uvicorn
    print("Starting ENGINE-TWIN Real-Time Server on http://127.0.0.1:8000")
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="info")
