# Phase 3 Implementation Plan — Hybrid Propulsion & Energy Storage Physics (Corrected)

This updated plan incorporates all 5 required corrections: explicit dimensional BSFC units, configurable battery reserve SOC, exact excess generator power control logic, and configurable generator maximum electrical rating limit.

---

## 7-Domain Power Flow Architecture

```
 ┌────────────────────────┐
 │ Engine Mechanical Pwr  │  (P_engine_mech in kW, [40 kW, 300 kW])
 └───────────┬────────────┘
             │  Generator Efficiency (eta_gen = 0.93) & max_elec limit (279 kW)
             ▼
 ┌────────────────────────┐
 │ Generator Electrical   │  (P_gen_elec = min(P_mech * eta_gen, P_gen_max_elec))
 └───────────┬────────────┘
             │
             ├───────────────────────────────────────────────┐
             │                                               │
             ▼                                               ▼
 ┌────────────────────────┐                    ┌────────────────────────┐
 │ DC Electrical Bus      │ ◄─────────────────►│ Battery Electrical Pwr │
 │ (P_gen + P_bat = P_bus)│                    │ (Discharge / Charge)   │
 └───────────┬────────────┘                    └────────────────────────┘
             │
             ▼
 ┌────────────────────────┐
 │ Motor Electrical Input │  (P_motor_elec <= 250.0 kW)
 └───────────┬────────────┘
             │  Motor Efficiency (eta_motor = 0.95)
             ▼
 ┌────────────────────────┐
 │ Motor Shaft Mechanical │  (P_motor_shaft <= 237.5 kW)
 └───────────┬────────────┘
             │  Propeller Efficiency (eta_prop = 0.85)
             ▼
 ┌────────────────────────┐
 │ Propulsive Output Pwr  │  (P_propulsive = Drag * V + m * g * Vz)
 └───────────┬────────────┘
```

---

## Key Refinements Incorporated

1. **Dimensional BSFC Formula:**
   $$\text{fuel\_used\_kg} = \frac{\text{power\_kw} \times \text{bsfc\_g\_per\_kwh} \times dt\_seconds}{3,600,000}$$
   For $BSFC_{\text{kg/kWh}} = 0.18\text{ kg/kWh} = 180\text{ g/kWh}$, this matches $\frac{\text{power\_kw} \times 180 \times dt\_seconds}{3,600,000}$.

2. **Configurable Battery Minimum Reserve SOC:**
   - `min_soc_percent` loaded from `battery_specs.json` (default 20%).
   - Represents mission reserve energy. When $SoC \le \text{min\_soc\_percent}$, available battery discharge power is $0.0\text{ kW}$.

3. **Exact Excess Generator Power Control Logic:**
   - When $P_{\text{gen\_elec}} > P_{\text{bus\_req}}$, supply $P_{\text{bus\_req}}$ first.
   - Charge battery with remaining excess electrical power $P_{\text{excess}} = P_{\text{gen\_elec}} - P_{\text{bus\_req}}$ up to $\min(P_{\text{batt\_chg\_max}}, P_{\text{cap\_remaining\_kw}})$.
   - Apply charging efficiency $\eta_{\text{charge}} = 0.95$.
   - Throttle engine mechanical power down if excess power cannot be stored in battery.

4. **Configurable Generator Electrical Limit:**
   - $P_{\text{gen\_elec}} = \min(P_{\text{engine\_mech}} \times \eta_{\text{gen}}, \text{generator\_max\_electrical\_kw})$.

---

## Files to Modify and Create

1. [`src/engine/engine.py`](file:///C:/Users/offic/.gemini/antigravity/scratch/Hybrid-Electric-UAV-Simulator/src/engine/engine.py): BSFC dimensional fuel formula, configurable generator max electrical rating limit, generator electrical output calculation.
2. [`src/battery/battery.py`](file:///C:/Users/offic/.gemini/antigravity/scratch/Hybrid-Electric-UAV-Simulator/src/battery/battery.py): Configurable `min_soc_percent` reserve enforcement, power/energy limited discharge and charge methods.
3. [`src/motor/motor.py`](file:///C:/Users/offic/.gemini/antigravity/scratch/Hybrid-Electric-UAV-Simulator/src/motor/motor.py): Separate electrical input power $P_{\text{elec}}$ and shaft output power $P_{\text{shaft}} = P_{\text{elec}} \times \eta_{\text{motor}}$, motor saturation tracking.
4. [`src/power_manager/power_manager.py`](file:///C:/Users/offic/.gemini/antigravity/scratch/Hybrid-Electric-UAV-Simulator/src/power_manager/power_manager.py): DC bus power conservation, excess generator power battery charging and engine throttling logic.
5. [`src/mission/simulator.py`](file:///C:/Users/offic/.gemini/antigravity/scratch/Hybrid-Electric-UAV-Simulator/src/mission/simulator.py): 7-domain power reporting, component saturation logging, conservation check.
6. [`tests/test_propulsion_physics.py`](file:///C:/Users/offic/.gemini/antigravity/scratch/Hybrid-Electric-UAV-Simulator/tests/test_propulsion_physics.py) **[NEW]**: Test suite for all 14 Phase 3 validation requirements.
7. `PHASE3_COMPLETION.md` **[NEW]**: Completion report.
