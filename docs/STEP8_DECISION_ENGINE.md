# STEP 8: AI-Guided Mission Decision Engine & Multi-Criteria Trajectory Optimizer

## Executive Summary
Step 8 elevates the **DRDO Mission-Aware UAV Digital Twin (`DRDO-UAV-EngineTwin`)** by incorporating an explainable, deterministic **AI-Guided Mission Decision Engine** and a physically grounded **Multi-Criteria Trajectory Optimizer**. The system continuously synthesizes the multi-subsystem virtual twin (MVEM piston engine, Battery Energy Storage System, parallel electric motor/generator, power management controller, and 3D spatial navigation state) with Step 7 reliability scoring to autonomously formulate flight recommendations:
1. `CONTINUE_MISSION`: Planned flight route can proceed safely within all reserve margins.
2. `ABORT_AND_RTB`: Regulatory reserve breached or non-critical anomaly detected; initiate orderly return to home base.
3. `DIVERT_TO_ALTERNATE`: Home base unreachable within safety margins; divert to designated recovery airfield.
4. `EMERGENCY_DESCENT`: Critical propulsion loss, thermal hazard, or total energy depletion; initiate controlled descent to safe terrain.

Parallel to decision formulation, the **Multi-Criteria Trajectory Optimizer** generates, simulates, and ranks candidate recovery paths (Direct RTB, Corridor RTB, Mission Continuation, and Alternate Airfield Diversions) using aerodynamic equations, hybrid powertrain mode simulation, hard safety reserve constraints, and a multi-criteria normalized cost function.

---

## Architecture & Module Structure

```
DRDO-UAV-EngineTwin/
├── configs/
│   ├── decision_engine_config.json      # [NEW] Policies, optimization weights, normalization scales, alternate airfields
│   ├── reliability_config.json          # Step 7 reliability thresholds & recovery bases
│   └── mission_config.json              # 7-waypoint flight route & home base
├── mission/
│   ├── __init__.py                      # [UPDATED] Step 8 symbols exported
│   ├── decision_engine.py               # [NEW] MissionDecisionEngine, DecisionEngineConfig, MissionDecisionResult
│   ├── trajectory_optimizer.py          # [NEW] TrajectoryOptimizer, CandidateRoute, TrajectoryOptimizationTelemetry
│   ├── reliability_engine.py            # Step 7 MissionReliabilityEngine & RTB predictor
│   ├── navigation.py                    # Spatial navigation engine & Haversine 3D formulas
│   └── mission_state.py                 # GeographicPosition, Waypoint, Subsystem states
├── digital_twin/
│   └── uav_digital_twin.py              # [UPDATED] Full integration, validation rules & history snapshotting
├── dashboard/
│   ├── index.html                       # [UPDATED] Tab 6: AI Decision & Route Optimizer + Top status tape
│   └── app.js                           # [UPDATED] Real-time 20 Hz telemetry parsing & candidate routes table
├── tests/
│   ├── test_mission_decision_engine.py  # [NEW] 12/12 Comprehensive validation scenarios
│   └── test_trajectory_optimizer.py     # [NEW] 7/7 Comprehensive trajectory optimization scenarios
├── test_mission_decision_engine.py      # Root test runner proxy
├── test_trajectory_optimizer.py         # Root test runner proxy
├── run_all_tests.py                     # [UPDATED] Consolidated test master runner (15 verification suites)
└── docs/
    └── STEP8_DECISION_ENGINE.md         # [NEW] Complete Step 8 technical dossier
```

---

## 1. Safety Reserve Policy & Configuration (`configs/decision_engine_config.json`)

The decision engine and trajectory optimizer share a strictly validated JSON configuration schema:

```json
{
  "policy_name": "DRDO-MALE-UAV-Decision-Engine-Standard-v1",
  "min_battery_reserve_pct": 20.0,
  "min_fuel_reserve_liters": 18.0,
  "emergency_power_reserve_w": 15000.0,
  "max_permitted_power_deficit_w": 8000.0,
  "critical_engine_health_threshold_pct": 40.0,
  "degraded_engine_health_threshold_pct": 65.0,
  "critical_battery_temp_c": 65.0,
  "caution_battery_temp_c": 55.0,
  "min_decision_confidence_healthy": 0.95,
  "telemetry_stale_timeout_s": 0.20,
  "optimization_interval_s": 1.0,
  "route_optimization_weights": {
    "weight_fuel": 0.30,
    "weight_battery": 0.20,
    "weight_duration": 0.20,
    "weight_distance": 0.10,
    "weight_margin": 0.10,
    "weight_degradation": 0.10
  },
  "normalization_scales": {
    "norm_distance_m": 300000.0,
    "norm_duration_s": 5400.0,
    "norm_fuel_l": 35.0,
    "norm_battery_wh": 18000.0
  },
  "alternate_destinations": [
    {
      "destination_id": "ALT-01",
      "name": "Baramati Airfield (VABB)",
      "latitude_deg": 18.2547,
      "longitude_deg": 74.5828,
      "altitude_m": 604.0,
      "runway_length_m": 1700.0,
      "priority": 1
    },
    {
      "destination_id": "ALT-02",
      "name": "Solapur Airport (VASL)",
      "latitude_deg": 17.6289,
      "longitude_deg": 75.9339,
      "altitude_m": 483.0,
      "runway_length_m": 2000.0,
      "priority": 2
    },
    {
      "destination_id": "ALT-03",
      "name": "Ahmednagar Airfield (VAAP)",
      "latitude_deg": 19.0833,
      "longitude_deg": 74.7500,
      "altitude_m": 655.0,
      "runway_length_m": 1500.0,
      "priority": 3
    }
  ]
}
```

### Schema Integrity Verification
Upon loading, `DecisionEngineConfig.validate()` executes mathematical validation checks:
- Optimization weights must sum to $1.0 \pm 0.01$ and contain zero negative values.
- Normalization scales ($d_{\text{norm}}, t_{\text{norm}}, f_{\text{norm}}, b_{\text{norm}}$) must be strictly positive.
- Safety thresholds must maintain logical ordering: $\text{critical} < \text{degraded}$ and $\text{caution} < \text{critical}$.
- Battery SoC reserve must reside strictly within $(0, 100)\%$.

---

## 2. Deterministic AI Decision Engine (`mission/decision_engine.py`)

The Decision Engine implements an explainable rule-based decision tree that evaluates platform safety reserves, propulsion availability, and situational trust.

### Explicit Handling of Missing Telemetry
The engine strictly rejects missing or unvalidated inputs:
```python
missing_fields = []
if timestamp_s is None: missing_fields.append("timestamp_s")
if quality_status is None: missing_fields.append("quality_status")
if engine_health_pct is None: missing_fields.append("engine_health_pct")
if battery_soc_pct is None: missing_fields.append("battery_soc_pct")
if remaining_fuel_liters is None: missing_fields.append("remaining_fuel_liters")

if missing_fields:
    return MissionDecisionResult(
        recommended_action="ABORT_AND_RTB",
        decision_reason=f"Critical telemetry inputs missing: {', '.join(missing_fields)}. Abort mandatory.",
        decision_confidence=0.10,
        triggered_conditions=["MISSING_CRITICAL_TELEMETRY"],
        mission_feasible=False,
        rtb_feasible=False,
        alternate_feasible=False,
        selected_destination="UNKNOWN_EMERGENCY",
        decision_timestamp=float(timestamp_s or 0.0),
        data_quality_status="INVALID"
    )
```

### Confidence Calibration
Confidence is calibrated from data quality and platform operational stress:
$$\text{Confidence} = C_{\text{nominal}} - \Delta C_{\text{quality}} - \Delta C_{\text{stale}} - \Delta C_{\text{deficit}} - \Delta C_{\text{health}}$$
- Nominal baseline: $0.95$
- Data Quality penalty: WARNING ($-0.10$), DEGRADED ($-0.35$), INVALID (fixed at $0.20$).
- Stale sensor lag: Clamped to $\le 0.55$.
- Deficit fraction penalty: Up to $-0.15 \times \frac{P_{\text{deficit}}}{P_{\text{req}}}$.
- Bounded strictly within $[0.10, 1.00]$.

### Prioritized Deterministic Decision Tree
1. **Rule 1 (Catastrophic Emergency):** Dual propulsion loss, battery thermal runaway ($T_{\text{batt}} \ge 65^\circ$C), or total energy depletion $\rightarrow$ `EMERGENCY_DESCENT`.
2. **Rule 2 (Data Corruption):** Telemetry quality `INVALID` $\rightarrow$ `ABORT_AND_RTB` with confidence clamped to $\le 0.40$.
3. **Rule 3 (Stale Data Lag):** Subsystem lag $> 200$ ms $\rightarrow$ `ABORT_AND_RTB` with confidence $\le 0.55$.
4. **Rule 4 (Home Base Infeasible):** Return to home base violates reserves, but an alternate is reachable $\rightarrow$ `DIVERT_TO_ALTERNATE`.
5. **Rule 5 (Reserve Policy Breach):** Fuel $< 18$ L or Battery SoC $< 20\%$ $\rightarrow$ `ABORT_AND_RTB`.
6. **Rule 6 (Mechanical Degradation):** Engine health $< 65\%$ $\rightarrow$ `ABORT_AND_RTB` or `DIVERT_TO_ALTERNATE`.
7. **Rule 7 (Persistent Power Deficit):** $P_{\text{deficit}} > 8$ kW $\rightarrow$ `ABORT_AND_RTB`.
8. **Rule 8 (Mission Route Infeasible):** Remaining route energy demand exceeds budget $\rightarrow$ `ABORT_AND_RTB`.
9. **Rule 9 (All Systems Nominal):** All safety and operational margins satisfied $\rightarrow$ `CONTINUE_MISSION` (confidence $\ge 0.95$).

---

## 3. Multi-Criteria Trajectory Optimizer (`mission/trajectory_optimizer.py`)

The trajectory optimizer generates, simulates, and ranks candidate paths without black-box heuristics or fictitious terrain grids.

### Aerodynamic Power Model
Propulsion power required for steady flight is derived from fundamental aerospace principles:
$$P_{\text{shaft}} = \frac{m \cdot g \cdot V}{(L/D) \cdot \eta_{\text{prop}}} + \frac{m \cdot g \cdot V_{\text{climb}}}{\eta_{\text{prop}}}$$
Where:
- UAV Mass $m = 1100\text{ kg}$
- Cruise $L/D = 15.0$
- Propeller efficiency $\eta_{\text{prop}} = 0.82$
- Nominal cruise airspeed $V = 55\text{ m/s}$ (producing nominal shaft demand $P_{\text{shaft}} \approx 48.24\text{ kW}$)

### Hybrid Powertrain Strategy Simulation
Along each candidate segment, the hybrid split is evaluated according to power demand and battery state:
- **`ENGINE_ONLY`**: Standard cruise. $P_{\text{eng}} = P_{\text{req}}$, $\dot{m}_{\text{fuel}} \approx 20\text{ LPH}$.
- **`MOTOR_ASSIST`**: High demand exceeding engine capacity. Motor buffers deficit up to $40\text{ kW}$.
- **`BATTERY_CHARGING`**: Low battery SoC ($< 28\%$) with surplus engine headroom. Engine generates an additional $20\text{ kW}$ to recharge BESS.
- **`ELECTRIC_ONLY`**: Quiet loiter or thermal emergency. Engine at idle, motor delivers full demand.

### Multi-Criteria Normalized Cost Function
Candidate trajectories are scored across six normalized physical metrics:
$$\text{Cost} = w_{\text{fuel}} m_{\text{fuel}} + w_{\text{batt}} m_{\text{batt}} + w_{\text{time}} m_{\text{time}} + w_{\text{dist}} m_{\text{dist}} + w_{\text{margin}} m_{\text{margin}} + w_{\text{deg}} m_{\text{deg}}$$

Where:
- $m_{\text{fuel}} = \min(2.0, \frac{\text{Fuel Burn}}{35.0\text{ L}})$
- $m_{\text{batt}} = \min(2.0, \frac{\text{Battery Wh}}{18000.0\text{ Wh}})$
- $m_{\text{time}} = \min(2.0, \frac{\text{Duration}}{5400.0\text{ s}})$
- $m_{\text{dist}} = \min(2.0, \frac{\text{Distance}}{300000.0\text{ m}})$
- $m_{\text{margin}} = 1.0 - \min(1.0, \frac{\text{Margin \%}}{100.0})$
- $m_{\text{deg}} = \left(1.0 - \frac{\text{Engine Health}}{100.0}\right) \times 1.5$

### Hard Safety Constraints Enforcement
A candidate route is flagged as **`is_feasible = False`** if:
1. Projected remaining fuel $< 18.0\text{ L}$.
2. Projected remaining battery SoC $< 20.0\%$.
3. Segment power deficit $> 8.0\text{ kW}$.
4. Negative energy buffer margin.

Infeasible routes receive a $+10.0$ cost penalty, guaranteeing that an unsafe shortcut can never rank above a safe path.

### Execution Throttling (20 Hz Safe)
Graph-search and multi-criteria evaluations are throttled at `optimization_interval_s = 1.0` seconds ($1\text{ Hz}$) and cached. High-frequency $20\text{ Hz}$ telemetry ticks immediately retrieve the cached result in $\mathcal{O}(1)$ time, eliminating CPU starvation.

---

## 4. Unified Digital Twin Integration (`digital_twin/uav_digital_twin.py`)

The Digital Twin state layer integrates Step 8 without altering earlier subsystem contracts:
- `UAVDigitalTwinState`: Added `decision: Optional[MissionDecisionResult]` and `trajectory: Optional[TrajectoryOptimizationTelemetry]`.
- `UAVDigitalTwinValidator`: Verifies numerical bounds and valid action enums. Trajectory staleness is permitted up to $2.0\text{ s}$ to honor $1\text{ Hz}$ optimization throttling without false staleness alerts.
- `update_from_hybrid_telemetry`: Coordinates navigation, reliability, decision engine, and trajectory optimizer in strict sequence.

---

## 5. Ground Control Station Integration (`dashboard/`)

The web-based Ground Control Station was updated with full Step 8 operational visibility:
- **Top Flight Avionics Bar**:
  - `badge-decision-action`: Shows current action (`CONTINUE_MISSION`, `ABORT_AND_RTB`, `DIVERT_TO_ALTERNATE`, `EMERGENCY_DESCENT`) with color coding.
  - `badge-route-opt`: Displays selected optimal trajectory ID.
- **Tab 6: AI Decision & Route Optimizer**:
  - **Deterministic Decision Engine Card**: Displays action banner, confidence percentage, human-readable rationale, and live triggered safety condition tags.
  - **Multi-Criteria Trajectory Optimizer Card**: Displays selected route summary (distance, time, fuel burn, battery kWh, reserve margin) and an interactive candidate route ranking table with normalized costs.

---

## 6. Verification & Test Results

All test suites were executed on Python 3.14 (64-bit) under Windows.

### Test Execution Summary
| Test Suite | Scenarios | Result | Status |
| :--- | :---: | :---: | :---: |
| `tests/test_mission_decision_engine.py` | 12 | 12/12 PASS | **100% OPERATIONAL** |
| `tests/test_trajectory_optimizer.py` | 7 | 7/7 PASS | **100% OPERATIONAL** |
| `test_uav_digital_twin.py` (Regression) | 6 | 6/6 PASS | **100% OPERATIONAL** |
| `test_mission_reliability.py` (Step 7) | 10 | 10/10 PASS | **100% OPERATIONAL** |
| `test_mission_navigation.py` (Step 5) | 8 | 8/8 PASS | **100% OPERATIONAL** |
| `test_hybrid_powertrain.py` (Step 3) | 6 | 6/6 PASS | **100% OPERATIONAL** |
| `test_battery.py` (Step 2) | 5 | 5/5 PASS | **100% OPERATIONAL** |
| `test_simulation.py` (Step 1) | 5 | 5/5 PASS | **100% OPERATIONAL** |
| **Consolidated Master Suite** | **59** | **59/59 PASS** | **100% VERIFIED** |

### Verified Test Cases in `test_mission_decision_engine.py`:
1. `test_1_normal_healthy_sortie`: Nominal flight produces `CONTINUE_MISSION` with confidence $\ge 0.95$.
2. `test_2_low_battery_reserve_violation`: Battery SoC at $15\%$ mandates `ABORT_AND_RTB`.
3. `test_3_critically_depleted_battery`: Battery SoC at $6.5\%$ and engine health at $35\%$ mandates `EMERGENCY_DESCENT`.
4. `test_4_low_fuel_reserve_violation`: Fuel at $14.5\text{ L}$ mandates `ABORT_AND_RTB`.
5. `test_5_degraded_engine_mechanical_health`: Engine health at $52\%$ aborts mission.
6. `test_6_insufficient_return_energy_to_home`: Unreachable home base routes aircraft to Baramati alternate.
7. `test_7_dual_propulsion_loss_emergency`: Total propulsion loss triggers `EMERGENCY_DESCENT`.
8. `test_8_critical_battery_overheat`: Battery temperature at $68.5^\circ$C triggers `EMERGENCY_DESCENT`.
9. `test_9_invalid_corrupted_telemetry`: NaN telemetry clobbers confidence to $\le 0.40$ and aborts mission.
10. `test_10_stale_telemetry_lag`: Telemetry lag $> 200\text{ ms}$ reduces confidence and aborts sortie.
11. `test_11_explicit_missing_critical_telemetry`: Missing inputs handled gracefully without crash.
12. `test_12_config_validation_and_safety_checks`: Catches non-normalizing weights and inverted thresholds.

### Verified Test Cases in `test_trajectory_optimizer.py`:
1. `test_1_candidate_routes_generation`: Generates Direct RTB, Corridor RTB, Mission Continue, and Alternates.
2. `test_2_aerodynamic_power_and_hybrid_strategy`: Validates $P_{\text{aero}}$ calculation and hybrid splits.
3. `test_3_hard_safety_reserve_constraints`: Strictly rejects paths breaching $18\text{ L}$ fuel or $20\%$ SoC.
4. `test_4_multi_criteria_cost_function`: Validates cost normalization across all 6 weighted components.
5. `test_5_dynamic_candidate_ranking`: Verifies rank ordering with feasible paths strictly preceding infeasible ones.
6. `test_6_emergency_scenario_no_feasible_route`: Detects zero-feasible route scenarios and flags status cleanly.
7. `test_7_execution_throttling_and_caching`: Caches optimization results at $1\text{ Hz}$ to protect the $20\text{ Hz}$ loop.
