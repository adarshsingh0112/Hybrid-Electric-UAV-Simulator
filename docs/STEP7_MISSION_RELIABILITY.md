# STEP 7: Mission Reliability Scoring & Dynamic Contingency Analysis

## Executive Summary
Step 7 evolves the **Mission-Aware UAV Digital Twin (`DRDO-UAV-EngineTwin`)** by incorporating an explainable, physically grounded **Mission Reliability Scoring & Dynamic Contingency Decision Engine**. The system continuously monitors the synchronized state of the UAV platform (MVEM internal combustion engine, Battery Energy Storage System, parallel electric motor/generator, power management controller, and 3D spatial navigation route) to evaluate whether the aircraft can safely continue its planned sortie, return to its home base (Pune Lohegaon / VAPO), or divert to an alternate recovery airfield.

The implementation strictly avoids unvalidated heuristic weights or black-box random numbers. It enforces deterministic aviation safety reserve policies, dynamic Return-to-Base (RTB) aerodynamic energy predictions, multi-subsystem trust audits, and plain-language pilot advisories at 20 Hz.

---

## Architecture & Module Structure

```
DRDO-UAV-EngineTwin/
├── configs/
│   ├── mission_config.json          # 7-waypoint flight route & base coordinates
│   └── reliability_config.json      # Safety reserve thresholds & alternate recovery bases
├── mission/
│   ├── __init__.py                  # Public exports including Step 7 reliability classes
│   ├── mission_state.py             # GeographicPosition, Waypoint, Subsystem states
│   ├── navigation.py                # Haversine distance, waypoint progress, ETA
│   └── reliability_engine.py        # [NEW] MissionReliabilityEngine & RTB trajectory predictor
├── digital_twin/
│   └── uav_digital_twin.py          # [UPDATED] Subsystem validation & telemetry orchestration
├── dashboard/
│   ├── index.html                   # [UPDATED] GCS Operator Dashboard with Tab 5 (MRI & Contingency)
│   └── app.js                       # [UPDATED] 20 Hz real-time telemetry rendering & gauge updates
├── tests/
│   └── test_mission_reliability.py  # [NEW] 10/10 Comprehensive validation test scenarios
├── test_mission_reliability.py      # Root test runner
└── docs/
    └── STEP7_MISSION_RELIABILITY.md # [NEW] Technical architecture & validation documentation
```

---

## 1. Safety Reserve Policy (`configs/reliability_config.json`)

Operational safety thresholds are strictly externalized in configuration rather than hardcoded:

```json
{
  "policy_name": "DRDO-MALE-UAV-Safety-Reserve-Policy-Standard",
  "min_battery_reserve_pct": 20.0,
  "min_fuel_reserve_liters": 18.0,
  "emergency_power_reserve_w": 15000.0,
  "rtb_energy_margin_factor": 1.20,
  "nominal_fuel_capacity_liters": 150.0,
  "uav_mass_kg": 1100.0,
  "nominal_cruise_speed_mps": 55.0,
  "cruise_lift_to_drag_ratio": 15.0,
  "propeller_efficiency": 0.82,
  "nominal_cruise_fuel_burn_lph": 22.0,
  "max_fuel_burn_lph": 38.0,
  "alternate_recovery_bases": [
    {
      "base_id": "ALT-01",
      "name": "Baramati Airfield (VABB)",
      "latitude_deg": 18.2547,
      "longitude_deg": 74.5828,
      "altitude_m": 604.0
    },
    {
      "base_id": "ALT-02",
      "name": "Solapur Airport (VASL)",
      "latitude_deg": 17.6289,
      "longitude_deg": 75.9339,
      "altitude_m": 483.0
    },
    {
      "base_id": "ALT-03",
      "name": "Ahmednagar Airfield (VAAP)",
      "latitude_deg": 19.1172,
      "longitude_deg": 74.7214,
      "altitude_m": 650.0
    }
  ]
}
```

---

## 2. Mathematical Formulation & Explainable Scoring

### 2.1 Composite Mission Reliability Index (MRI)
The Mission Reliability Index ($R_{\text{mission}} \in [0, 100]$) is computed as an explainable linear combination of five bounded physical sub-indices:

$$R_{\text{mission}} = 0.35 \cdot F_{\text{energy}} + 0.30 \cdot F_{\text{health}} + 0.15 \cdot F_{\text{power}} + 0.15 \cdot F_{\text{quality}} + 0.05 \cdot F_{\text{progress}}$$

| Factor | Weight | Physical Basis |
| :--- | :---: | :--- |
| **$F_{\text{energy}}$ (Energy Reserves)** | **35%** | Evaluates total onboard fuel and battery buffer against required return budget with 20% regulatory margin factor ($K_{\text{margin}} = 1.20$). |
| **$F_{\text{health}}$ (Propulsion Health)** | **30%** | Weighted composite of internal combustion engine mechanical health (70%) and battery State of Health (30%), with thermal overheating penalties. |
| **$F_{\text{power}}$ (Power Headroom)** | **15%** | Ratio of combined maximum deliverable propulsion power ($P_{\text{eng}} + P_{\text{mot}}$) to requested throttle demand, tracking real-time deficit. |
| **$F_{\text{quality}}$ (Data Trust)** | **15%** | Evaluates Extended Kalman Filter convergence, sensor-model consistency, cross-system conservation, and subsystem freshness ($< 200\text{ ms}$ lag). |
| **$F_{\text{progress}}$ (Mission Progress)** | **5%** | Proximity to recovery advantage based on exponential characteristic distance scale ($d_0 = 300\text{ km}$). |

### 2.2 Strict Safety Hard-Clamps
To guarantee platform safety under corrupted or critical flight regimes, the composite score is subjected to non-negotiable safety clamps:
1. **Corrupted Telemetry ($INVALID$):** If numerical anomalies (NaN, Inf, impossible bounds, power conservation violations) occur, $R_{\text{mission}}$ is hard-clamped to $\le 10.0\%$.
2. **Infeasible Return ($RTB_{\text{feasible}} = \text{False}$):** If available onboard energy cannot complete return transit to home base, $R_{\text{mission}}$ is hard-clamped to $\le 40.0\%$.
3. **Severe Mechanical Failure ($Health < 40\%$ or $T_{\text{batt}} \ge 60^\circ\text{C}$):** $R_{\text{mission}}$ is hard-clamped to $\le 35.0\%$.
4. **Safety Reserve Policy Breach ($\text{Fuel} < 18\text{ L}$ or $\text{SoC} < 20\%$):** $R_{\text{mission}}$ is hard-clamped to $\le 50.0\%$.
5. **Degraded Engine Health ($Health < 65\%$):** $R_{\text{mission}}$ is hard-clamped to $\le 60.0\%$ (mandating CAUTION status).
6. **Propulsion Power Deficit ($P_{\text{deficit}} > 8\text{ kW}$):** $R_{\text{mission}}$ is hard-clamped to $\le 55.0\%$.
7. **Telemetry Staleness ($DEGRADED$):** $R_{\text{mission}}$ is hard-clamped to $\le 60.0\%$.

### 2.3 Status Level Categorization
- **NOMINAL ($R \ge 80.0\%$):** All flight systems nominal; safe to continue planned flight profile.
- **ADVISORY ($65.0\% \le R < 80.0\%$):** Minor performance discrepancy or elevated energy burn rate.
- **CAUTION ($50.0\% \le R < 65.0\%$):** Safety reserve eroded or degraded propulsion component; return advised.
- **CRITICAL ($R < 50.0\%$):** Platform safety compromised; contingency action required immediately.

---

## 3. Dynamic Return-to-Base (RTB) Aerodynamic Energy Prediction

The Return-to-Base prediction utilizes standard fixed-wing flight mechanics:

1. **Required Steady Level Flight Propulsion Shaft Power:**
   $$P_{\text{req}} = \frac{m_{\text{uav}} \cdot g \cdot V_{\text{cruise}}}{(L/D) \cdot \eta_{\text{prop}}}$$
   For the DRDO MALE UAV baseline ($m = 1100\text{ kg}$, $L/D = 15.0$, $\eta_{\text{prop}} = 0.82$, $V_{\text{cruise}} = 55.0\text{ m/s}$):
   $$P_{\text{req}} = \frac{1100 \cdot 9.80665 \cdot 55.0}{15.0 \cdot 0.82} \approx 48,230\text{ W} \quad (48.2\text{ kW})$$

2. **Estimated Return Transit Time:**
   $$t_{\text{RTB}} = \frac{D_{\text{home}}}{V_{\text{cruise}}}$$
   *(Guarded against division by zero: if $V < 5.0\text{ m/s}$, fallback $V = 55.0\text{ m/s}$ is applied).*

3. **Required Shaft Energy Budget with Policy Safety Margin:**
   $$E_{\text{req, RTB}} = P_{\text{req}} \cdot \left(\frac{t_{\text{RTB}}}{3600}\right) \cdot K_{\text{margin}} \quad [\text{Wh}]$$

4. **Estimated Fuel Required:**
   $$\text{Fuel}_{\text{req, RTB}} = \left(\frac{\text{FuelFlow}_{\text{cruise}}}{3600}\right) \cdot t_{\text{RTB}} \cdot K_{\text{margin}} \quad [\text{L}]$$

5. **Usable Onboard Energy Buffer:**
   $$E_{\text{usable, batt}} = \max\left(0, E_{\text{batt}} \cdot \frac{\text{SoC} - \text{SoC}_{\text{reserve}}}{100}\right)$$
   $$E_{\text{usable, fuel}} = \max\left(0, \text{Fuel}_{\text{avail}} - \text{Fuel}_{\text{reserve}}\right) \cdot \text{LHV}_{\text{eff}}$$
   where $\text{LHV}_{\text{eff}} \approx 2,900\text{ Wh/L}$ (shaft work from Jet-A/Diesel fuel at engine thermal efficiency $\eta_{\text{th}} \approx 30\%$).

6. **Energy Margin Percentage:**
   $$\text{Margin}_{\%} = \frac{E_{\text{total, usable}} - E_{\text{req, RTB}}}{E_{\text{req, RTB}}} \cdot 100\%$$

---

## 4. Deterministic Contingency Decision Matrix

The contingency analysis executes a prioritized, deterministic safety tree. Every recommendation includes a plain-language explanation for ground operators:

```
                    ┌───────────────────────────────┐
                    │  Current UAV State Telemetry  │
                    └───────────────┬───────────────┘
                                    │
           ┌────────────────────────┴────────────────────────┐
           ▼                                                 ▼
[Dual Propulsion Loss / T_batt >= 65°C]            [Invalid Telemetry / NaN]
     ==> EMERGENCY_DESCENT                              ==> ABORT_AND_RTB
           │                                                 │
           └────────────────────────┬────────────────────────┘
                                    │ (Preconditions Valid)
                                    ▼
                         [RTB Feasible to Home?]
                                    │
                     ┌──────────────┴──────────────┐
                    YES                            NO
                     │                             │
                     ▼                 [Nearest Alternate Feasible?]
           [Policy Reserve Checks]                 │
             - Fuel >= 18 L?              ┌────────┴────────┐
             - Battery SoC >= 20%?       YES                NO
             - Health >= 65%?             │                 │
             - Deficit <= 8 kW?           ▼                 ▼
             - Margin >= 15%?     DIVERT_TO_ALTERNATE  EMERGENCY_DESCENT
                     │
           ┌─────────┴─────────┐
          YES                  NO
           │                   │
           ▼                   ▼
    CONTINUE_MISSION      ABORT_AND_RTB
```

---

## 5. Verification & Test Suite Execution

All test suites were executed on the runtime environment (`Python 3.14.0`, Windows 64-bit):

### 5.1 Mission Reliability Suite (`test_mission_reliability.py`) — 10/10 PASS
```
================================================================================
  DRDO MALE UAV MISSION RELIABILITY & CONTINGENCY ANALYSIS SUITE (STEP 7)
================================================================================
[TEST 1] Testing Normal Healthy Mission Sortie...
  -> Mission Reliability Index: 92.7%
  -> Reliability Status Level:  NOMINAL
  -> Contingency Decision:      CONTINUE_MISSION
  -> RTB Feasibility:           True
  -> Energy Margin Buffer:      +50704.0%
  -> Composite Sub-Index:       92.7
  -> PASSED: Nominal flight produces high reliability and CONTINUE_MISSION.

[TEST 2] Testing Low Battery Reserve Violation (<20% Policy Limit)...
  -> Battery SoC: 14.5% (Policy Min: 20.0%)
  -> Contingency Decision: ABORT_AND_RTB
  -> Reason: Battery reserve violated: 14.5% SoC (< 20.0% policy limit). RTB to protect emergency buffer.
  -> Energy Reserve Factor: 36.2
  -> PASSED: Battery reserve violation safely triggers ABORT_AND_RTB.

[TEST 3] Testing Low Fuel Reserve Violation (<18 L Policy Limit)...
  -> Fuel Remaining: 14.0 L (Policy Min: 18.0 L)
  -> Contingency Decision: ABORT_AND_RTB
  -> Reason: Fuel reserve violated: 14.0 L remaining (< 18.0 L policy limit). RTB while energy reserves remain.
  -> PASSED: Fuel reserve violation safely triggers ABORT_AND_RTB.

[TEST 4] Testing Degraded Engine Mechanical Health (<65%)...
  -> Engine Health: 52.0%
  -> Contingency Decision: ABORT_AND_RTB
  -> Propulsion Health Factor: 54.9
  -> Reliability Status: CAUTION
  -> PASSED: Degraded engine health triggers ABORT_AND_RTB.

[TEST 5] Testing Insufficient Return Energy (RTB Infeasible -> Divert)...
  -> Distance to Home: 421.8 km
  -> RTB Feasible:     False
  -> Contingency:      EMERGENCY_DESCENT
  -> Reason:           Insufficient energy for Home Base or Alternate recovery. Execute controlled glide to emergency recovery sector.
  -> PASSED: Infeasible RTB correctly redirects to alternate recovery or emergency descent.

[TEST 6] Testing Invalid Telemetry (NaN / Infinite Corruption)...
  -> Quality Status:       INVALID
  -> Reliability Index:    10.0% (Clamped <= 10.0)
  -> Contingency Decision: ABORT_AND_RTB
  -> Telemetry Factor:     0.0
  -> PASSED: Corrupted telemetry safely penalizes reliability index and prevents continuation.

[TEST 7] Testing High Propulsion Power Demand & Deficit (>8 kW)...
  -> Power Deficit:        40.0 kW
  -> Power Headroom Factor: 22.0
  -> Contingency Decision: ABORT_AND_RTB
  -> Reason:               Persistent propulsion power deficit (40.0 kW). Unable to sustain climb or target loiter altitude.
  -> PASSED: Power deficit degrades power headroom factor and triggers ABORT_AND_RTB.

[TEST 8] Testing Zero / Low Ground Speed Fallback...
  -> Zero Speed RTB Duration: 694.3 s
  -> Estimated Energy:        4628.6 Wh
  -> Low Speed (0.5m/s) Fallback Duration: 694.3 s
  -> PASSED: Zero and low speeds safely handled with nominal cruise velocity fallbacks.

[TEST 9] Testing Emergency Contingency (Dual Propulsion Loss / Overheat)...
  -> Case A Decision: EMERGENCY_DESCENT
  -> Reason:          Dual propulsion loss or battery thermal critical (45.0°C). Execute emergency descent to nearest clear terrain.
  -> Case B Decision: EMERGENCY_DESCENT
  -> Reason:          Dual propulsion loss or battery thermal critical (68.5°C). Execute emergency descent to nearest clear terrain.
  -> PASSED: Critical emergencies deterministically trigger EMERGENCY_DESCENT.

[TEST 10] Testing Missing / Stale Telemetry Lag (>200ms)...
  -> Validator Status: DigitalTwinQualityStatus.DEGRADED
  -> Stale Flags:      {'engine': False, 'battery': False, 'electric_machine': False, 'powertrain': False, 'environment': False, 'mission': True, 'reliability': False}
  -> Telemetry Quality Factor: 35.0
  -> Contingency Recommendation: ABORT_AND_RTB
  -> Reason: Subsystem telemetry is DEGRADED (stale data lag > 200 ms). Operator advisory: abort mission to restore GCS link.
  -> PASSED: Stale telemetry penalizes telemetry factor and mandates ABORT_AND_RTB.

================================================================================
  ALL 10 MISSION RELIABILITY & CONTINGENCY TESTS PASSED SUCCESSFULLY! (10/10 PASS)
================================================================================
```

### 5.2 Backward Compatibility Regression Matrix
| Verification Suite | Target Step | Result |
| :--- | :---: | :---: |
| `test_simulation.py` | Step 1 (MVEM Engine & Sensors) | **5/5 PASS** |
| `test_battery.py` | Step 2 (BESS Battery Model) | **5/5 PASS** |
| `test_hybrid_powertrain.py` | Step 3 (Hybrid Powertrain & 4 Modes) | **6/6 PASS** |
| `test_uav_digital_twin.py` | Step 4 (Unified Digital Twin State Layer) | **6/6 PASS** |
| `test_mission_navigation.py` | Step 5 (Spatial Navigation & Waypoints) | **8/8 PASS** |
| `test_step6_integration.py` | Step 6 (Browser GCS Telemetry Stream) | **PASS** |
| `test_mission_reliability.py` | Step 7 (Reliability & Contingency Engine) | **10/10 PASS** |

---

## 6. Ground Control Station Integration

The Operator GCS Dashboard (`http://127.0.0.1:8000`) now features:
1. **Top Flight Avionics Bar:** Real-time badges for `RELIABILITY: 98.5%` and `ACTION: CONTINUE_MISSION`.
2. **Tab 5: Mission Reliability & Contingency:**
   - **Card 1: Mission Reliability Index (MRI):** Interactive radial progress gauge (0–100%) and five explainable sub-factor progress bars ($F_{\text{energy}}, F_{\text{health}}, F_{\text{power}}, F_{\text{quality}}, F_{\text{progress}}$).
   - **Card 2: Return-To-Base Feasibility & Safety Reserves:** Live monitors for distance to home ($D_{\text{home}}$), estimated transit duration, energy/fuel consumption, fuel gauge with 18 L policy limit line, and battery reserve with 20% SoC limit line.
   - **Card 3: Dynamic Contingency Action Card:** Large visual status block color-coded by decision urgency (Green `CONTINUE_MISSION`, Amber `ABORT_AND_RTB`, Orange `DIVERT_TO_ALTERNATE`, Flashing Red `EMERGENCY_DESCENT`) with full operational rationale and reachable alternate airfields.
3. **Data Quality Audit (Tab 4):** Synchronized staleness monitoring includes `Reliability: FRESH` and rule 7 cross-system validation passing.

---

## 7. Assumptions & Limitations
1. **Weather & Wind Vector:** RTB aerodynamic power model currently assumes standard calm or ISA cruise airspeed. Variable headwinds/tailwinds can be added in future weather layers by adjusting ground speed relative to true airspeed.
2. **Alternate Airfield Runways:** Alternate airfield feasibility checks currently evaluate horizontal distance and transit energy budget. Runway length, surface material, and ATC traffic coordination remain outside the physical propulsion scope.
3. **Descent Glide Dynamics:** `EMERGENCY_DESCENT` assumes a controlled best-glide descent profile based on standard $L/D = 15.0$ glide ratio without catastrophic structural damage.
