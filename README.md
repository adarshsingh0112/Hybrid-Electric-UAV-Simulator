# 🛩️ Hybrid-Electric UAV Simulator & Telemetry Dashboard

[![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)](https://www.python.org/)
[![Streamlit](https://img.shields.io/badge/Streamlit-Dashboard-FF4B4B.svg)](https://streamlit.io/)
[![Pytest](https://img.shields.io/badge/Tests-53%20Passed-brightgreen.svg)](https://docs.pytest.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A high-fidelity **continuous fixed-time-step physical simulation engine** and **interactive dark aerospace mission control dashboard** for a fixed-wing Hybrid-Electric Unmanned Aerial Vehicle (UAV).

---

## 📌 Project Overview

This simulator provides continuous state-propagation dynamics and 7-domain hybrid propulsion energy analytics for long-endurance UAV operations. It models the physical interactions between atmospheric ambient conditions, aircraft aerodynamics, turboshaft engine generation, Li-ion battery storage, electric motor drive, and continuous gross mass decay.

```
       ┌────────────────┐
       │ ISA Atmosphere │ (Altitude, Temperature, Density ρ)
       └───────┬────────┘
               │
               ▼
┌──────────────────────────────┐
│  Aircraft Flight Dynamics   │ (Lift, Drag Polar, Airspeed & Acceleration Limits)
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│  DC Bus Electrical Demand   │ (P_required = P_thrust / η_total)
└──────────────┬───────────────┘
               │
      ┌────────┴────────┐
      ▼                 ▼
┌───────────┐     ┌───────────┐
│ Generator │  +  │ Battery   │  =  Required DC Bus Power
└───────────┘     └───────────┘
 (Turboshaft)     (Li-ion SoC)
```

---

## 🌟 Key Features & Phase Milestones

### 1. Core Physical Conservation & Architecture (Phase 1)
- **Conservation of Power**: Enforces strict electrical bus balance:
  $$\mathbf{P_{\text{engine\_electric}} + P_{\text{battery}} = P_{\text{required}}}$$
- **Clamped Limits Handling**: Automatically reallocates power deficit/excess when components hit physical operating limits (e.g. engine idle/max or battery min SOC).

### 2. Aerodynamics & ISA Atmosphere Model (Phase 2)
- **ISA Atmospheric Physics**: Temperature, pressure, and density profiles computed as functions of altitude $h$:
  $$\rho(h) = \rho_0 \left(1 - \frac{L \cdot h}{T_0}\right)^{\frac{g \cdot M}{R \cdot L}}$$
- **Quadratic Drag Polar**: Lift coefficient $C_L$ and dynamic drag $C_D = C_{D0} + K C_L^2$.
- **3-Tier Efficiency Chain**: Total efficiency $\eta_{\text{total}} = \eta_{\text{motor}} \cdot \eta_{\text{propeller}} \cdot \eta_{\text{inverter}}$.

### 3. Hybrid Energy Storage & Dimensional Fuel Physics (Phase 3)
- **Dimensional BSFC Formula**: Exact fuel mass consumption calculation:
  $$\Delta m_{\text{fuel}} = \frac{P_{\text{engine\_kW}} \cdot \text{BSFC}_{\text{g/kWh}} \cdot \Delta t_{\text{s}}}{3,600,000}$$
- **Battery Energy Reserve Protection**: Usable discharge halts when State of Charge (SoC) drops to the configurable reserve limit ($20\%$).
- **Generator Power Flow**: Turboshaft generator rated at $279\text{ kW}$ continuous ($300\text{ kW}$ max). Excess power automatically recharges the battery or throttles down.

### 4. Continuous Fixed-Time-Step Engine (Phase 4)
- **State Propagation Loop**: Fixed time-step state update ($\Delta t = 1.0\text{ s}$) using an exact-duration timer loop (`while phase_elapsed_s < phase_duration_s`).
- **Dynamic Aircraft Mass Decay**: Total vehicle gross mass calculated at every timestep:
  $$m_{\text{aircraft}}(t) = m_{\text{dry}} (650) + m_{\text{payload}} (200) + m_{\text{battery}} (520) + m_{\text{fuel}}(t)$$
- **Kinematic Constraints**: Kinematic acceleration bounds ($+1.0\text{ m/s}^2$ max climb acceleration, $-0.5\text{ m/s}^2$ max descent rate).
- **Power Deficit Logging**: Tracks cumulative deficit energy ($E_{\text{deficit\_kWh}}$) and power-limited duration without halting execution.

### 5. Interactive Dark Aerospace Mission Control Dashboard (Phase 5)
- **Mission Control HUD Header**: Live telemetry status indicator, system badges, and metadata.
- **Aircraft Control Panel Sidebar**: Interactive resolution selector ($\Delta t = 0.5\text{s}, 1.0\text{s}, 2.0\text{s}, 5.0\text{s}$), live simulation rerun trigger, and 2-column hardware specs grid.
- **Glassmorphism Executive Metric Cards**: 12 summary metrics with hover elevation and dynamic mission status banners (**MISSION SUCCESS** / **POWER LIMITED**).
- **Interactive Plotly Visualizations**:
  - ✈️ **Flight Kinematics**: Altitude, Airspeed, Battery SoC, Fuel Mass, Vehicle Mass.
  - ⚡ **5-Channel Power Flow**: DC Bus Demand, Generator Output, Battery Power, Motor Input, Power Deficit.
  - ⏱️ **Mission Phase Timeline**: Interactive Gantt chart for phase durations.
  - ⚠️ **Power Deficit Analysis**: Instantaneous deficit energy plot & feasibility metrics.
  - 📋 **Telemetry Explorer**: Data table view with CSV download capability.

---

## 📊 Aircraft & Propulsion Specifications

| Component / Parameter | Value / Specification | Description |
| :--- | :--- | :--- |
| **Max Takeoff Mass (MTOW)** | **1,620 kg** | Initial gross mass at mission start |
| **Dry Vehicle Mass** | **650 kg** | Airframe and structural dry weight |
| **Payload Capacity** | **200 kg** | Sensor package and avionics payload |
| **Battery Pack Mass** | **520 kg** | Li-ion battery pack constant mass |
| **Initial Fuel Mass** | **250 kg** | Jet-A usable fuel capacity |
| **Battery Storage Capacity** | **220 kWh** | Nominal energy capacity |
| **Minimum Battery SoC Reserve**| **20.0 %** | Protected reserve threshold |
| **Turboshaft Engine Peak** | **300.0 kW** | Maximum short-term shaft power |
| **Generator Electrical Rating**| **279.0 kW** | Continuous electrical power limit |
| **Electric Motor Peak Power** | **250.0 kW** | Continuous motor electrical input limit |

---

## 📁 Repository Structure

```text
Hybrid-Electric-UAV/
├── .streamlit/
│   └── config.toml                  # Dark aerospace Streamlit theme configuration
├── data/
│   └── uav_specs.json               # System configuration & hardware specifications
├── logs/
│   └── mission_log.csv              # High-resolution time-series telemetry (21,000 records)
├── src/
│   ├── aircraft/                    # Aircraft aerodynamic state & mass calculations
│   ├── battery/                     # Li-ion battery pack energy & SoC dynamics
│   ├── controller/                  # Hybrid UAV main flight execution wrapper
│   ├── dashboard/                   # Terminal ASCII progress bar renderer
│   ├── engine/                      # Turboshaft engine & fuel consumption physics
│   ├── environment/                 # ISA Standard Atmosphere model (T, p, ρ)
│   ├── mission/                     # Mission phase manager & continuous simulator engine
│   ├── motor/                       # Electric motor drive & efficiency models
│   ├── optimization/                # Energy optimization utility placeholders
│   ├── power_manager/               # Conservation-of-power distribution logic
│   ├── telemetry/                   # Time-series telemetry logger & CSV exporter
│   └── main.py                      # Terminal simulation execution entry point
├── tests/                           # Pytest unit & integration test suite (53 tests)
│   ├── test_aerodynamics.py
│   ├── test_atmosphere.py
│   ├── test_battery.py
│   ├── test_continuous_simulation.py
│   ├── test_dashboard.py
│   ├── test_engine.py
│   ├── test_power_conservation.py
│   ├── test_propulsion_physics.py
│   └── test_simulation.py
├── uav_dashboard/                   # Streamlit Interactive Dashboard Package
│   ├── components/                  # UI visual components (sidebar, cards, charts, timeline)
│   ├── utils/                       # Data loader, validation, and metrics calculation
│   ├── dashboard.py                 # Forwarding entry module
│   └── styles.py                    # CSS dark aerospace design system & Plotly theme
├── app.py                           # Streamlit Web Application Entry Point
├── .gitignore                       # Git ignore rules for caches & temporary files
├── pytest.ini                       # Pytest execution configuration
└── README.md                        # Documentation
```

---

## 🚀 Quick Start & Usage

### 1. Prerequisites & Installation

Ensure you have Python 3.9+ installed. Clone the repository and install required packages:

```bash
git clone https://github.com/adarshsingh0112/Hybrid-Electric-UAV-Simulator.git
cd Hybrid-Electric-UAV-Simulator
pip install streamlit pandas numpy plotly pytest
```

### 2. Run Terminal Simulation

Execute the continuous time-step simulation from the command line:

```bash
python src/main.py
```

This generates or updates the high-resolution mission telemetry in `logs/mission_log.csv`.

### 3. Launch Interactive Mission Control Dashboard

Start the Streamlit dashboard:

```bash
python -m streamlit run app.py
```

Open your browser at **`http://localhost:8501`** to interact with the dashboard.

---

## 🧪 Testing & Verification

Run the full pytest suite to verify physical conservation laws, ISA calculations, continuous state propagation, and dashboard metrics:

```bash
python -m pytest
```

**Test Results:**
```text
============================= test session starts =============================
collected 53 items

tests/test_aerodynamics.py ......                                       [ 11%]
tests/test_atmosphere.py ...                                            [ 16%]
tests/test_battery.py ...                                               [ 22%]
tests/test_continuous_simulation.py ..............                      [ 49%]
tests/test_dashboard.py ......                                          [ 60%]
tests/test_engine.py ....                                               [ 67%]
tests/test_power_conservation.py ...                                    [ 73%]
tests/test_propulsion_physics.py .............                          [ 98%]
tests/test_simulation.py .                                              [100%]

======================= 53 passed in 286.78s (0:04:46) ========================
```

---

## 📜 License

Distributed under the MIT License. See `LICENSE` for details.
