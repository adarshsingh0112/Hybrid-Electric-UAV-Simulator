import sys
from pathlib import Path
import streamlit as st

# Ensure src is in pythonpath
src_path = Path(__file__).resolve().parent.parent.parent / "src"
if str(src_path) not in sys.path:
    sys.path.append(str(src_path))


def render_sidebar() -> float:
    """Renders dashboard sidebar for simulation timestep selection, rerun execution, and parameters display.

    Returns:
        float: Selected simulation timestep dt_s in seconds.
    """
    st.sidebar.markdown(
        """
        <div style="text-align: center; padding: 10px 0 15px 0;">
            <div style="font-size: 2.2rem; line-height: 1.2;">✈️</div>
            <div style="font-size: 1.1rem; font-weight: 700; color: #F8FAFC; letter-spacing: 0.5px; margin-top: 4px;">
                MISSION CONTROL
            </div>
            <div style="font-size: 0.75rem; color: #38BDF8; font-weight: 600; letter-spacing: 1px;">
                HYBRID UAV SIMULATOR
            </div>
        </div>
        """,
        unsafe_allow_html=True
    )
    st.sidebar.markdown("---")

    st.sidebar.markdown("### ⏱️ Simulation Settings")

    preset = st.sidebar.selectbox(
        "Timestep Preset Resolution",
        options=["1.0 s (Default)", "0.5 s", "2.0 s", "5.0 s", "Custom"],
        index=0,
        help="Select fixed time-step dt resolution for continuous state propagation."
    )

    if preset == "1.0 s (Default)":
        dt_s = 1.0
    elif preset == "0.5 s":
        dt_s = 0.5
    elif preset == "2.0 s":
        dt_s = 2.0
    elif preset == "5.0 s":
        dt_s = 5.0
    else:
        dt_s = st.sidebar.slider(
            "Custom Timestep dt (s)",
            min_value=0.1,
            max_value=5.0,
            value=1.0,
            step=0.1,
            help="Higher resolution (smaller dt) increases simulation data points."
        )

    st.sidebar.caption(f"Configured Resolution: **dt = {dt_s:.2f} s**")

    # Run Simulation Button
    if st.sidebar.button("🚀 Run New Simulation", type="primary", use_container_width=True):
        with st.spinner(f"Running continuous simulation (dt = {dt_s:.2f} s)..."):
            from controller.hybrid_uav import HybridUAV
            uav = HybridUAV()
            uav.simulator.dt_s = float(dt_s)
            uav.fly()

            # Clear Streamlit cache so fresh log data is loaded immediately
            st.cache_data.clear()
            st.toast("Simulation Completed Successfully!", icon="✅")
            st.rerun()

    st.sidebar.markdown("---")

    # Aircraft & System Hardware Overview (Read-Only Grid HUD)
    st.sidebar.markdown("### 🛩️ Aircraft Specs")
    st.sidebar.markdown(
        """
        <div class="uav-specs-box">
            <div style="font-weight: 700; color: #F8FAFC; border-bottom: 1px solid rgba(56, 189, 248, 0.15); padding-bottom: 4px; margin-bottom: 6px;">
                Fixed-Wing Hybrid UAV
            </div>
            <div class="uav-specs-grid">
                <div class="uav-spec-item">
                    <span class="uav-spec-label">MTOW</span>
                    <span class="uav-spec-val">1620 kg</span>
                </div>
                <div class="uav-spec-item">
                    <span class="uav-spec-label">Dry Mass</span>
                    <span class="uav-spec-val">650 kg</span>
                </div>
                <div class="uav-spec-item">
                    <span class="uav-spec-label">Payload</span>
                    <span class="uav-spec-val">200 kg</span>
                </div>
                <div class="uav-spec-item">
                    <span class="uav-spec-label">Battery Mass</span>
                    <span class="uav-spec-val">520 kg</span>
                </div>
                <div class="uav-spec-item">
                    <span class="uav-spec-label">Init Fuel</span>
                    <span class="uav-spec-val">250 kg</span>
                </div>
                <div class="uav-spec-item">
                    <span class="uav-spec-label">Battery Cap</span>
                    <span class="uav-spec-val">220 kWh</span>
                </div>
                <div class="uav-spec-item">
                    <span class="uav-spec-label">Engine Max</span>
                    <span class="uav-spec-val">300 kW</span>
                </div>
                <div class="uav-spec-item">
                    <span class="uav-spec-label">Motor Max</span>
                    <span class="uav-spec-val">250 kW</span>
                </div>
            </div>
        </div>
        """,
        unsafe_allow_html=True
    )

    st.sidebar.markdown("---")
    st.sidebar.caption("Hybrid UAV Simulator v5.0 | Continuous Physics Engine")

    return float(dt_s)
