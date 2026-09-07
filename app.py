import os
import sys
from pathlib import Path
import streamlit as st
import pandas as pd

# Add src and current directory to pythonpath
BASE_DIR = Path(__file__).resolve().parent
SRC_DIR = BASE_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

from uav_dashboard.styles import inject_custom_css
from uav_dashboard.utils.data_loader import (
    load_telemetry,
    validate_telemetry,
    calculate_dashboard_metrics,
)
from uav_dashboard.components.sidebar import render_sidebar
from uav_dashboard.components.summary_cards import render_summary_cards
from uav_dashboard.components.charts import (
    render_kinematics_charts,
    render_power_flow_chart,
)
from uav_dashboard.components.timeline import render_phase_timeline
from uav_dashboard.components.power_deficit import render_power_deficit_section

# Streamlit Page Config
st.set_page_config(
    page_title="Hybrid UAV Simulator Dashboard",
    page_icon="🛩️",
    layout="wide",
    initial_sidebar_state="expanded"
)


def main():
    # Inject Dark Aerospace CSS Theme
    inject_custom_css()

    # 1. Render Sidebar & retrieve selected timestep
    dt_s = render_sidebar()

    # Hero / Header HUD Section
    st.markdown(
        """
        <div class="uav-hero-container">
            <div style="display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 12px;">
                <div>
                    <h1 class="uav-hero-title">🛩️ Hybrid Electric UAV Simulator</h1>
                    <div class="uav-hero-subtitle">
                        <span>Continuous Time-Step Physical Telemetry</span>
                        <span>•</span>
                        <span>ISA Atmosphere Model</span>
                        <span>•</span>
                        <span>7-Domain Energy Analytics</span>
                    </div>
                </div>
                <div>
                    <span class="status-badge">
                        <span class="status-pulse-dot"></span> LIVE TELEMETRY SYSTEM
                    </span>
                </div>
            </div>
        </div>
        """,
        unsafe_allow_html=True
    )

    # 2. Check telemetry file existence and mtime for safe caching
    log_path = BASE_DIR / "logs" / "mission_log.csv"

    if not log_path.exists():
        st.warning("⚠️ Mission telemetry log file (`logs/mission_log.csv`) not found.")
        st.info("Run the continuous time-step simulation first using the sidebar button or command line:")
        st.code("python src/main.py")
        st.stop()

    # Retrieve file modification timestamp to invalidate stale cache automatically
    mtime = os.path.getmtime(log_path)

    try:
        df = load_telemetry(str(log_path), mtime)
    except Exception as e:
        st.error(f"❌ Error loading telemetry data: {e}")
        st.stop()

    # 3. Validate Telemetry Data Integrity
    is_valid, err_msg = validate_telemetry(df)
    if not is_valid:
        st.error(f"❌ Telemetry Data Validation Error: {err_msg}")
        st.info("Please rerun the simulation to generate clean telemetry data.")
        st.stop()

    # 4. Calculate Aggregate Metrics & Status
    metrics = calculate_dashboard_metrics(df)

    # 5. Executive Summary Metric Cards
    render_summary_cards(metrics)
    st.divider()

    # 6. Tabbed Visualization Sections
    tab1, tab2, tab3, tab4, tab5 = st.tabs([
        "✈️ Flight Kinematics",
        "⚡ Power System Breakdown",
        "⏱️ Mission Phase Timeline",
        "⚠️ Power Deficit Analysis",
        "📋 Telemetry Explorer"
    ])

    with tab1:
        render_kinematics_charts(df)

    with tab2:
        render_power_flow_chart(df)

    with tab3:
        render_phase_timeline(df)

    with tab4:
        render_power_deficit_section(df, metrics)

    with tab5:
        st.markdown("### 📋 High-Resolution Telemetry Data Explorer")
        st.write(f"**Total Records:** {len(df):,} rows | **Total Attributes:** {len(df.columns)} columns")

        # CSV Download Button
        csv_bytes = df.to_csv(index=False).encode("utf-8")
        st.download_button(
            label="📥 Download Telemetry CSV",
            data=csv_bytes,
            file_name="hybrid_uav_mission_log.csv",
            mime="text/csv",
            type="primary"
        )

        st.dataframe(df, use_container_width=True, height=400)


if __name__ == "__main__":
    main()