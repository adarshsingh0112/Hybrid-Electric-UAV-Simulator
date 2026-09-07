import streamlit as st
from typing import Dict, Any


def render_summary_cards(metrics: Dict[str, Any]):
    """Renders key mission metrics using custom aerospace metric cards and status banners."""
    if not metrics:
        st.warning("No summary metrics available.")
        return

    # Status HUD Banner
    status = metrics.get("mission_status", "UNKNOWN")
    color = metrics.get("status_color", "blue")

    if color == "green":
        hud_class = "success"
        icon = "✅"
        badge_text = "NOMINAL OPERATIONAL PROFILE"
    elif color == "orange":
        hud_class = "warning"
        icon = "⚠️"
        badge_text = "POWER DEFICIT DETECTED • QUASI-STEADY LIMIT"
    else:
        hud_class = "error"
        icon = "❌"
        badge_text = "CRITICAL MISSION INFEASIBILITY"

    st.markdown(
        f"""
        <div class="uav-status-hud {hud_class}">
            <div>
                <span style="font-size: 0.8rem; font-weight: 700; text-transform: uppercase; letter-spacing: 0.8px;">
                    MISSION STATUS EVALUATION
                </span>
                <div style="font-size: 1.35rem; font-weight: 700; margin-top: 2px;">
                    {icon} {status}
                </div>
            </div>
            <div>
                <span style="font-size: 0.75rem; font-weight: 600; padding: 4px 12px; border-radius: 20px; border: 1px solid currentColor;">
                    {badge_text}
                </span>
            </div>
        </div>
        """,
        unsafe_allow_html=True
    )

    st.markdown("### 📊 Mission Executive Summary")

    # Row 1 Cards: Duration, Mass, Data Volume
    col1, col2, col3, col4 = st.columns(4)
    col1.metric(
        "⏱️ Total Flight Duration",
        f"{metrics.get('duration_min', 0.0):.2f} min",
        delta=f"{metrics.get('duration_s', 0.0):.0f} sec total",
        delta_color="off"
    )
    col2.metric(
        "⚖️ Initial Gross Mass",
        f"{metrics.get('initial_mass_kg', 0.0):.1f} kg",
        help="Gross Mass = Dry (650) + Payload (200) + Battery (520) + Fuel (250 kg)"
    )
    col3.metric(
        "🛬 Final Aircraft Mass",
        f"{metrics.get('final_mass_kg', 0.0):.1f} kg",
        delta=f"-{metrics.get('fuel_consumed_kg', 0.0):.2f} kg fuel",
        delta_color="normal"
    )
    col4.metric(
        "📈 Telemetry Records",
        f"{metrics.get('total_rows', 0):,}",
        help="High-resolution time-series data rows generated."
    )

    # Row 2 Cards: Fuel & Battery Reserves
    col5, col6, col7, col8 = st.columns(4)
    col5.metric(
        "⛽ Fuel Consumed",
        f"{metrics.get('fuel_consumed_kg', 0.0):.2f} kg",
        help="Total Jet-A fuel consumed during mission."
    )
    col6.metric(
        "🛢️ Fuel Remaining",
        f"{metrics.get('final_fuel_kg', 0.0):.2f} kg",
        help="Remaining fuel mass in tank at mission completion."
    )
    col7.metric(
        "🔋 Final Battery SoC",
        f"{metrics.get('final_soc_percent', 0.0):.1f}%",
        help="Usable energy reserve protected at 20% min SOC."
    )
    col8.metric(
        "⚡ Peak Req Elec Power",
        f"{metrics.get('max_req_power_kw', 0.0):.1f} kW",
        help="Maximum electrical power requested on DC bus."
    )

    # Row 3 Cards: Generator, Battery Power & Deficit Energy
    col9, col10, col11, col12 = st.columns(4)
    col9.metric(
        "⚙️ Peak Generator Power",
        f"{metrics.get('max_gen_power_kw', 0.0):.1f} kW",
        help="Peak electrical output from turboshaft generator."
    )
    col10.metric(
        "🔌 Peak Battery Discharge",
        f"{metrics.get('max_bat_power_kw', 0.0):.1f} kW",
        help="Peak electrical power discharged from Li-ion battery."
    )
    col11.metric(
        "⚠️ Total Deficit Energy",
        f"{metrics.get('total_deficit_energy_kwh', 0.0):.4f} kWh",
        help="Cumulative electrical power deficit energy."
    )
    col12.metric(
        "🚨 Power Limited Duration",
        f"{metrics.get('power_limited_seconds', 0.0):.1f} s",
        help="Total continuous time spent in power deficit condition."
    )
