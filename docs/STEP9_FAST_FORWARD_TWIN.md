# STEP 9: Multi-Scenario Predictive Simulation (Fast-Forward Digital Twin)

## Executive Summary
Step 9 advances the **DRDO Mission-Aware UAV Digital Twin (`DRDO-UAV-EngineTwin`)** from reactive telemetry monitoring and instantaneous trajectory evaluation to **Faster-Than-Real-Time Multi-Scenario Predictive Simulation**. 

Operating as a virtual "fast-forward" twin, this capability projects candidate flight trajectories into the future across an ensemble of physical uncertainty realizations. By simulating hours of flight time in tens of milliseconds, the system calculates empirical mission success probabilities, reserve margin percentiles ($p_{05}, p_{50}, p_{95}$), and safety limit violation frequencies before any physical aircraft command is committed.

### Primary Capabilities:
1. **Model State Isolation**: Cloned, independent physical model instances execute all forward projections, guaranteeing zero mutation or drift of the live Digital Twin state.
2. **Physics-Grounded Uncertainty Modeling**: Perturbations explicitly model the atmospheric wind vector triangle, temperature deviations from standard ISA, mechanical engine wear/derating, battery equivalent series resistance (ESR) growth, state estimation error, and airframe mass variations.
3. **Statistical Aggregation with Confidence Intervals**: Calculates completion rates, reserve violation frequencies, empirical percentiles ($p_{05}, p_{50}, p_{95}$), and 95% Wilson score confidence intervals.
4. **Multi-Route Comparative Ranking**: Evaluates all candidate recovery routes (Direct RTB, Corridor RTB, Diversions, Continuation) under identical scenario seeds, strictly enforcing that lower nominal cost cannot override hard safety constraints.
5. **GCS Dashboard Integration (Tab 7)**: Provides operators with real-time risk gauges, reserve uncertainty envelopes, and comparative route metrics alongside mandatory aerospace prediction disclaimers.

---

## Architecture & Module Structure

```
DRDO-UAV-EngineTwin/
├── configs/
│   ├── fast_forward_config.json         # [NEW] Monte Carlo distributions, bounds, budget & safety thresholds
│   ├── decision_engine_config.json      # Step 8 policies, alternate bases & weights
│   └── mission_config.json              # 7-waypoint flight route & home base
├── simulation/
│   ├── scenario_generator.py            # [NEW] ScenarioParameters, FastForwardConfig, ScenarioGenerator
│   ├── fast_forward_engine.py           # [NEW] FastForwardSimulationEngine, ScenarioSimulationRecord
│   ├── battery.py                       # Physical equivalent circuit BESS model (reused)
│   ├── hybrid_powertrain.py             # Hybrid powertrain & energy management controller (reused)
│   └── mvem.py                          # Calibrated Mean Value Engine Model (reused)
├── mission/
│   ├── __init__.py                      # [UPDATED] Step 9 symbols exported
│   ├── predictive_risk_analyzer.py      # [NEW] PredictiveRiskAnalyzer, RouteRiskSummary, PredictiveRiskReport
│   ├── trajectory_optimizer.py          # [UPDATED] Caches candidate routes for predictive evaluation
│   ├── decision_engine.py               # Step 8 AI Decision Engine
│   └── navigation.py                    # Spatial navigation engine & Haversine 3D formulas
├── digital_twin/
│   └── uav_digital_twin.py              # [UPDATED] Periodic 5.0s evaluation, validation rules, flat telemetry
├── dashboard/
│   ├── index.html                       # [UPDATED] Tab 7: Predictive Simulation & Risk Analysis
│   └── app.js                           # [UPDATED] Real-time 20 Hz telemetry parsing, gauges & candidate matrix
├── tests/
│   ├── test_scenario_generator.py       # [NEW] 6/6 Comprehensive scenario generator tests
│   ├── test_fast_forward_engine.py      # [NEW] 5/5 Comprehensive physics engine tests
│   └── test_predictive_risk_analyzer.py # [NEW] 5/5 Comprehensive statistical aggregation tests
├── test_scenario_generator.py           # Root test runner proxy
├── test_fast_forward_engine.py          # Root test runner proxy
├── test_predictive_risk_analyzer.py     # Root test runner proxy
├── run_all_tests.py                     # [UPDATED] Master test runner (18 verification suites)
└── docs/
    └── STEP9_FAST_FORWARD_TWIN.md       # [NEW] Complete Step 9 technical dossier
```

---

## 1. Monte Carlo Uncertainty Distributions (`simulation/scenario_generator.py`)

The scenario generator samples physically grounded perturbations from documented probability distributions configured in `configs/fast_forward_config.json`.

### Uncertainty Factors & Physical Justifications:
| Parameter | Distribution | Bounds / Nominal | Aeronautical Justification |
| :--- | :--- | :--- | :--- |
| **Wind Speed ($V_w$)** | Truncated Gaussian ($\mu=0, \sigma=4.0\text{ m/s}$) | $[0.0, 15.0]\text{ m/s}$ | Atmospheric boundary layer turbulence & synoptic front variability. |
| **Wind Direction ($\theta_w$)** | Uniform | $[0^\circ, 360^\circ)$ | Ambient winds can blow from any compass bearing. |
| **Ambient Temp ($\Delta T$)** | Truncated Gaussian ($\mu=0, \sigma=2.5^\circ\text{C}$) | $[-8.0, +8.0]^\circ\text{C}$ | Diurnal temperature cycles and non-standard ISA deviations. |
| **Engine Max Power ($k_P$)** | Uniform | $[0.85, 1.00]$ | Mechanical cylinder friction, turbocharger wear, injector fouling. |
| **Engine Fuel Flow ($k_F$)** | Uniform | $[1.00, 1.10]$ | Specific fuel consumption (SFC) deterioration over flight hours. |
| **Battery Resistance ($k_R$)** | Uniform | $[0.95, 1.30]$ | Pack aging, internal Joule heating, and cell thermal gradient. |
| **Battery SoC Offset ($\Delta\text{SoC}$)**| Uniform | $[-2.0\%, +2.0\%]$ | 12-state Extended Kalman Filter state estimation uncertainty. |
| **Propulsion Power ($k_{\text{req}}$)** | Uniform | $[0.95, 1.08]$ | Airframe surface roughness, bug accumulation, and control trimming. |
| **Payload Mass ($\Delta m$)** | Uniform | $[-25.0, +25.0]\text{ kg}$ | Avionics pod variations, fuel density uncertainty, ballast fitment. |

### Deterministic & Stochastic Ensembles:
1. **Nominal Scenario (`SCENARIO_NOMINAL`)**: Zero perturbations ($V_w=0$, $\Delta T=0$, all multipliers $= 1.0$).
2. **Best-Case Scenario (`SCENARIO_BEST_CASE`)**: Favorable tailwind (+5 m/s aligned with bearing), dense cool air ($-5^\circ\text{C}$), fresh powertrain (100% engine power, 0.98x fuel burn, 0.95x battery resistance).
3. **Worst-Case Scenario (`SCENARIO_WORST_CASE`)**: Severe opposing headwind (15 m/s), hot air ($+8^\circ\text{C}$), degraded engine (0.85x power, 1.10x fuel burn), aged battery (1.30x resistance, $-2.0\%$ initial SoC offset).
4. **Monte Carlo Ensemble (`SCENARIO_MC_xxx`)**: Configurable count (default: 20 scenarios) sampled deterministically using a fixed pseudo-random seed (`random_seed=42`).

---

## 2. Fast-Forward Physics Engine (`simulation/fast_forward_engine.py`)

The fast-forward engine simulates candidate trajectories forward in time faster than real time using isolated physical models.

### Mathematical Formulation:
1. **Wind Vector Triangle**:
   For each route segment with track azimuth $\theta_{\text{track}}$ and true airspeed $V_{\text{TAS}}$ (nominal $45.0\text{ m/s}$):
   $$\theta_{\text{rel}} = \theta_{\text{track}} - \theta_{\text{wind}}$$
   $$V_{\text{headwind}} = V_w \cdot \cos(\theta_{\text{rel}}), \quad V_{\text{crosswind}} = V_w \cdot \sin(\theta_{\text{rel}})$$
   $$V_{\text{ground}} = \sqrt{V_{\text{TAS}}^2 - V_{\text{crosswind}}^2} - V_{\text{headwind}}$$
   $$t_{\text{seg}} = \frac{D_{\text{seg}}}{V_{\text{ground}}}$$

2. **Aerodynamic Power Demand**:
   $$m_{\text{total}} = m_{\text{dry}} + m_{\text{payload}} + \Delta m + m_{\text{fuel}}(t)$$
   $$P_{\text{level}} = \frac{m_{\text{total}} \cdot g \cdot V_{\text{TAS}}}{(L/D) \cdot \eta_{\text{prop}}}, \quad P_{\text{climb}} = \frac{m_{\text{total}} \cdot g \cdot \max(0, \dot{h})}{\eta_{\text{prop}}}$$
   $$P_{\text{req}} = (P_{\text{level}} + P_{\text{climb}}) \cdot k_{\text{req}}$$

3. **Hybrid Powertrain Operating Modes**:
   - `ENGINE_ONLY`: Engine supplies $P_{\text{req}}$, battery idle.
   - `MOTOR_ASSIST`: Engine operates at maximum continuous rating; electric motor supplies deficit up to 40 kW.
   - `BATTERY_CHARGING`: Engine runs above propulsion demand to recharge battery through the electric generator.
   - `ELECTRIC_ONLY`: Engine throttled to idle; electric motor delivers silent/emergency propulsion.

4. **Lumped Thermal Dynamics**:
   Aligned with the calibrated Mean Value Engine Model (`simulation/mvem.py`):
   $$\text{CHT}_{\text{target}} = 110.0 + 85.0 \left(\frac{P_{\text{eng}}}{200\text{ HP}}\right) k_F + 0.25 (T_{\text{amb}} - 15) - 0.35 (V_{\text{TAS}} - 30)$$
   $$\frac{d\text{CHT}}{dt} = \frac{\text{CHT}_{\text{target}} - \text{CHT}}{\tau_{\text{CHT}}}, \quad \tau_{\text{CHT}} = 15.0\text{ s}$$

5. **Threshold Tracking**:
   Monitors the first timestamp at which any safety envelope is violated:
   - Fuel reserve breach: Fuel remaining $< 18.0\text{ L}$.
   - Battery reserve breach: SoC $< 20.0\%$.
   - Engine thermal breach: $\text{CHT} \ge 220.0^\circ\text{C}$.
   - Battery thermal breach: $T_{\text{batt}} \ge 65.0^\circ\text{C}$.
   - Uncompensated power deficit: $P_{\text{deficit}} > 8000\text{ W}$.

---

## 3. Predictive Risk Analyzer & Ensemble Statistics (`mission/predictive_risk_analyzer.py`)

### 1. Empirical Percentiles:
Percentiles ($p_{05}, p_{50}, p_{95}$) for flight duration, fuel remaining, battery SoC, and total reserve margin are computed using linear interpolation over the sorted scenario results:
$$k = (N - 1) \cdot \frac{p}{100}, \quad v_p = v_{\lfloor k \rfloor} + (k - \lfloor k \rfloor)(v_{\lceil k \rceil} - v_{\lfloor k \rfloor})$$

### 2. Wilson Score 95% Confidence Intervals:
For binomial proportions (such as mission completion rate with sample size $N$ and successes $s$):
$$\tilde{p} = \frac{s + \frac{z^2}{2N}}{1 + \frac{z^2}{N}}, \quad \text{margin} = \frac{z \sqrt{\frac{p(1-p)}{N} + \frac{z^2}{4N^2}}}{1 + \frac{z^2}{N}}, \quad z = 1.96$$
$$\text{CI}_{95\%} = [\max(0, \tilde{p} - \text{margin}), \min(1, \tilde{p} + \text{margin})]$$
*The Wilson score interval provides rigorous coverage for small Monte Carlo samples ($N=20$) and extreme probabilities ($p \to 0$ or $p \to 1$), where standard normal Wald approximations fail.*

### 3. Multi-Route Ranking & Safety Precedence:
Candidate routes are sorted into statistically feasible and infeasible cohorts. A route is rejected if:
- Ensemble completion rate $< 85.0\%$.
- Fuel reserve violation rate $> 20.0\%$.
- Battery depletion probability $> 5.0\%$.
- Thermal violation rate $> 15.0\%$.

Feasible routes are scored by:
$$\text{Score} = 0.40 \cdot \text{Rate}_{\text{comp}} + 0.30 \cdot \text{Margin}_{p50} + 0.15 \cdot (100 - \text{Rate}_{\text{fuel\_vio}}) + 0.15 \cdot (100 - \text{Rate}_{\text{thermal}})$$

---

## 4. Digital Twin & Telemetry Integration (`digital_twin/uav_digital_twin.py`)

- **Throttled Execution**: Predictive simulations execute periodically every **5.0 seconds** (configurable), avoiding CPU starvation while keeping predictions fresh.
- **Timestamp Synchronization**: Simulation time `master_t` is passed explicitly to `PredictiveRiskReport` to ensure zero clock skew.
- **Staleness Tolerance**: Digital Twin validator allows up to **10.0 seconds** of prediction age before flagging telemetry as degraded.
- **Exported Telemetry Schema**:
  - `prediction_status`: `"FEASIBLE"`, `"CAUTION"`, or `"NO_FEASIBLE_ROUTE"`
  - `prediction_scenario_count`: Total simulated runs (e.g., 23)
  - `predicted_mission_completion_rate`: Empirical % completion
  - `predicted_rtb_feasibility_rate`: Empirical % RTB feasibility
  - `predicted_battery_depletion_rate`: Risk of complete battery exhaustion
  - `predicted_engine_thermal_risk` & `predicted_battery_thermal_risk`
  - `predicted_fuel_reserve_violation_rate`: Risk of falling below 18L reserve
  - `predicted_energy_reserve_p05` & `predicted_energy_reserve_p95`: 5th & 95th percentile arrival reserves
  - `recommended_predictive_route`: Recommended candidate trajectory identifier
  - `route_summaries`: Array of comparative route records

---

## 5. Ground Control Station Operator UI (`dashboard/`)

### Tab 7: Predictive Simulation & Risk Analysis
1. **Aerospace Disclaimer Banner**: Prominent caution header stating predictions represent simulated physics-based ensembles under modeled uncertainty, not certified avionics.
2. **Circular Risk Gauges**:
   - Ensemble Completion Rate (%)
   - RTB Feasibility Rate (%)
   - Battery Depletion Probability (%)
   - Thermal Violation Risk (%)
3. **Arrival Reserve Uncertainty ($p_{05}, p_{50}, p_{95}$)**: Displays 5th percentile worst-case, 50th percentile median, and 95th percentile best-case arrival reserves for fuel and battery SoC.
4. **Candidate Route Comparison Matrix**: Displays comparative rankings, completion rates, 95% Wilson confidence intervals, duration percentiles, and statistical feasibility status for all candidate routes under identical conditions.

---

## 6. Verification & Test Coverage (17/17 PASS)

Three dedicated test suites rigorously validate Step 9:

### 1. Scenario Generator Suite (`tests/test_scenario_generator.py`):
- `test_1_nominal_scenario`: Baseline zero perturbations and unit multipliers.
- `test_2_bounded_best_and_worst_cases`: Verified tailwind vs headwind and degradation envelopes.
- `test_3_fixed_seed_reproducibility`: 100% deterministic bitwise matching across runs with seed 42.
- `test_4_monte_carlo_distributions_and_clipping`: Enforces strict physical clipping bounds across 50 MC draws.
- `test_5_full_ensemble_generation`: Validates composition (1 nominal, 2 bounds, $N$ MC scenarios).
- `test_6_configuration_validation_and_errors`: Schema validation and rejection of invalid values.
- **Result: 6/6 PASS**

### 2. Fast-Forward Engine Suite (`tests/test_fast_forward_engine.py`):
- `test_1_model_state_isolation`: Verified live battery and powertrain models remain unmutated.
- `test_2_battery_depletion_and_reserve_breach`: Validated battery reserve breach detection.
- `test_3_thermal_limit_breach_detection`: Validated engine CHT and battery thermal limit crossings.
- `test_4_fuel_reserve_violation_detection`: Validated detection of fuel reserve breach on extended flights.
- `test_5_faster_than_real_time_speed_benchmark`: Executed single flight routes in ~8.2 ms ($> 400\times$ faster than real time).
- **Result: 5/5 PASS**

### 3. Predictive Risk Analyzer Suite (`tests/test_predictive_risk_analyzer.py`):
- `test_1_statistical_percentiles_and_wilson_ci`: Validated linear interpolation percentiles and Wilson CI bounds.
- `test_2_route_ensemble_evaluation`: Validated ensemble metrics across 13 scenarios.
- `test_3_multi_route_comparison_identical_scenarios`: Verified deterministic ranking under identical uncertainty.
- `test_4_hard_safety_reserve_precedence`: Verified that safety reserve violations override lower nominal cost.
- `test_5_telemetry_snapshot_and_disclaimer`: Validated telemetry structure and mandatory aerospace disclaimers.
- **Result: 5/5 PASS**

---

## 7. Aerospace Certification & Regulatory Boundaries

> [!WARNING]
> **SIMULATION & DISCLAIMER NOTICE (DRDO SIH26054)**
> 1. **Empirical Statistics**: All completion rates, confidence bounds, and violation frequencies reported by the Fast-Forward Twin represent empirical metrics generated across simulated physical models under specified disturbance distributions.
> 2. **Not Certified DO-178C / DO-254 Probabilities**: These metrics do not substitute for formal civil or military airworthiness certification (e.g. ARP4754A, DO-178C DAL-A, or MIL-HDBK-516C).
> 3. **Pilot/Operator Authority**: The Fast-Forward Digital Twin provides decision-support advisory telemetry to the Ground Control Station operator. Final flight authority remains strictly with the pilot-in-command / mission commander.
