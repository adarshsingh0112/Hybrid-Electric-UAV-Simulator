import pandas as pd
import plotly.graph_objects as go
import streamlit as st
from typing import Dict, Any
from uav_dashboard.styles import apply_dark_theme


def render_power_deficit_section(df: pd.DataFrame, metrics: Dict[str, Any]):
    """Renders power deficit status banner, metrics, and deficit visualization plot."""
    st.markdown("### ⚠️ Power Deficit & Bus Feasibility Analysis")

    limited_seconds = metrics.get("power_limited_seconds", 0.0)
    deficit_energy = metrics.get("total_deficit_energy_kwh", 0.0)
    max_deficit = metrics.get("max_deficit_kw", 0.0)

    if limited_seconds <= 1e-3 and deficit_energy <= 1e-4 and max_deficit <= 1e-3:
        st.markdown(
            """
            <div class="uav-status-hud success">
                <div>
                    <span style="font-weight: 700; text-transform: uppercase;">✅ NO POWER DEFICIT DETECTED</span>
                    <div style="font-size: 0.88rem; margin-top: 4px; opacity: 0.9;">
                        All DC bus electrical demand was fully satisfied by engine generator output and battery discharge across the entire mission trajectory.
                    </div>
                </div>
            </div>
            """,
            unsafe_allow_html=True
        )
        return

    st.markdown(
        """
        <div class="uav-status-hud warning">
            <div>
                <span style="font-weight: 700; text-transform: uppercase;">⚠️ POWER DEFICIT OCCURRED DURING MISSION</span>
                <div style="font-size: 0.88rem; margin-top: 4px; opacity: 0.9;">
                    Total electrical bus demand exceeded available generator electrical rating and usable battery discharge power.
                </div>
            </div>
        </div>
        """,
        unsafe_allow_html=True
    )

    col1, col2, col3 = st.columns(3)
    col1.metric("🚨 Total Power Limited Duration", f"{limited_seconds:.1f} s")
    col2.metric("⚠️ Total Deficit Energy", f"{deficit_energy:.4f} kWh")
    col3.metric("📉 Maximum Power Deficit Peak", f"{max_deficit:.2f} kW")

    st.info(
        "**Note on Quasi-Steady Commanded Kinematics:**\n"
        "Phase 4/5 simulator tracks and quantifies power deficit energy without halting execution. "
        "Closed-loop flight envelope degradation (e.g. altitude loss or speed loss) is deferred to future flight dynamics phases."
    )

    # Plot Deficit vs Time
    time_col = "Mission Time (min)"
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=df[time_col], y=df["Power Deficit (kW)"],
        mode="lines", name="Power Deficit (kW)",
        line=dict(color="#EF4444", width=2),
        fill="tozeroy", fillcolor="rgba(239, 68, 68, 0.25)"
    ))

    fig.update_layout(
        title="Instantaneous Electrical Power Deficit Over Time (kW)",
        xaxis_title="Mission Time (min)",
        yaxis_title="Deficit Power (kW)",
        hovermode="x unified"
    )
    apply_dark_theme(fig)

    st.plotly_chart(fig, use_container_width=True)
