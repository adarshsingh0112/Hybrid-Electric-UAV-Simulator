import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
import streamlit as st
from uav_dashboard.styles import apply_dark_theme


def render_kinematics_charts(df: pd.DataFrame):
    """Renders interactive Plotly charts for mission kinematics: Altitude, Airspeed, SoC, Fuel, and Mass."""
    st.markdown("### ✈️ Kinematics & State Evolution")

    time_col = "Mission Time (min)"

    col_left, col_right = st.columns(2)

    # 1. Altitude vs Time
    with col_left:
        fig_alt = go.Figure()
        fig_alt.add_trace(go.Scatter(
            x=df[time_col], y=df["Altitude (m)"],
            mode="lines", name="Actual Altitude (m)",
            line=dict(color="#38BDF8", width=2)
        ))
        if "Target Altitude (m)" in df.columns:
            fig_alt.add_trace(go.Scatter(
                x=df[time_col], y=df["Target Altitude (m)"],
                mode="lines", name="Target Altitude (m)",
                line=dict(color="#0EA5E9", width=1.5, dash="dash")
            ))
        fig_alt.update_layout(
            title="Altitude Profile (m)",
            xaxis_title="Mission Time (min)",
            yaxis_title="Altitude (m)",
            hovermode="x unified",
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
        )
        apply_dark_theme(fig_alt)
        st.plotly_chart(fig_alt, use_container_width=True)

    # 2. Airspeed vs Time
    with col_right:
        fig_speed = go.Figure()
        speed_col = "Airspeed (km/h)" if "Airspeed (km/h)" in df.columns else "Speed (km/h)"
        fig_speed.add_trace(go.Scatter(
            x=df[time_col], y=df[speed_col],
            mode="lines", name="Actual Speed (km/h)",
            line=dict(color="#34D399", width=2)
        ))
        if "Target Speed (km/h)" in df.columns:
            fig_speed.add_trace(go.Scatter(
                x=df[time_col], y=df["Target Speed (km/h)"],
                mode="lines", name="Target Speed (km/h)",
                line=dict(color="#F87171", width=1.5, dash="dash")
            ))
        fig_speed.update_layout(
            title="Airspeed Profile (km/h)",
            xaxis_title="Mission Time (min)",
            yaxis_title="Speed (km/h)",
            hovermode="x unified",
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
        )
        apply_dark_theme(fig_speed)
        st.plotly_chart(fig_speed, use_container_width=True)

    st.markdown("---")

    col_left2, col_right2 = st.columns(2)

    # 3. Battery SoC vs Time
    with col_left2:
        fig_soc = go.Figure()
        fig_soc.add_trace(go.Scatter(
            x=df[time_col], y=df["Battery SoC (%)"],
            mode="lines", name="Battery SoC (%)",
            line=dict(color="#C084FC", width=2)
        ))
        # 20% Reserve SoC Limit Line
        fig_soc.add_hline(
            y=20.0, line_dash="dot", line_color="#EF4444",
            annotation_text="20% Reserve Limit", annotation_position="bottom right"
        )
        fig_soc.update_layout(
            title="Battery State of Charge (%)",
            xaxis_title="Mission Time (min)",
            yaxis_title="SoC (%)",
            yaxis=dict(range=[0, 105]),
            hovermode="x unified"
        )
        apply_dark_theme(fig_soc)
        st.plotly_chart(fig_soc, use_container_width=True)

    # 4. Fuel Remaining vs Time
    with col_right2:
        fig_fuel = go.Figure()
        fig_fuel.add_trace(go.Scatter(
            x=df[time_col], y=df["Fuel Remaining (kg)"],
            mode="lines", name="Fuel Remaining (kg)",
            line=dict(color="#F59E0B", width=2)
        ))
        fig_fuel.update_layout(
            title="Fuel Remaining (kg)",
            xaxis_title="Mission Time (min)",
            yaxis_title="Fuel Mass (kg)",
            hovermode="x unified"
        )
        apply_dark_theme(fig_fuel)
        st.plotly_chart(fig_fuel, use_container_width=True)

    # 5. Aircraft Mass vs Time
    fig_mass = go.Figure()
    fig_mass.add_trace(go.Scatter(
        x=df[time_col], y=df["Aircraft Mass (kg)"],
        mode="lines", name="Aircraft Total Mass (kg)",
        line=dict(color="#E879F9", width=2)
    ))
    fig_mass.update_layout(
        title="Aircraft Total Mass Evolution (kg)",
        xaxis_title="Mission Time (min)",
        yaxis_title="Total Vehicle Mass (kg)",
        hovermode="x unified"
    )
    apply_dark_theme(fig_mass)
    st.plotly_chart(fig_mass, use_container_width=True)


def render_power_flow_chart(df: pd.DataFrame):
    """Renders 5-channel interactive Plotly chart for hybrid power system breakdown."""
    st.markdown("### ⚡ Hybrid Propulsion Power Flow")

    time_col = "Mission Time (min)"

    fig = go.Figure()

    # 1. Required Motor Electrical Power
    fig.add_trace(go.Scatter(
        x=df[time_col], y=df["Required Elec Power (kW)"],
        mode="lines", name="DC Bus Required Power (kW)",
        line=dict(color="#38BDF8", width=2.5)
    ))

    # 2. Generator Electrical Power
    fig.add_trace(go.Scatter(
        x=df[time_col], y=df["Generator Power (kW)"],
        mode="lines", name="Generator Electrical Power (kW)",
        line=dict(color="#34D399", width=2)
    ))

    # 3. Battery Electrical Power (+Discharge / -Charge)
    fig.add_trace(go.Scatter(
        x=df[time_col], y=df["Battery Power (kW)"],
        mode="lines", name="Battery Power (kW) [+Dis/-Chg]",
        line=dict(color="#F59E0B", width=2)
    ))

    # 4. Motor Electrical Power
    if "Motor Elec Power (kW)" in df.columns:
        fig.add_trace(go.Scatter(
            x=df[time_col], y=df["Motor Elec Power (kW)"],
            mode="lines", name="Motor Elec Power In (kW)",
            line=dict(color="#C084FC", width=1.5, dash="dot")
        ))

    # 5. Power Deficit (kW)
    if "Power Deficit (kW)" in df.columns:
        fig.add_trace(go.Scatter(
            x=df[time_col], y=df["Power Deficit (kW)"],
            mode="lines", name="Power Deficit (kW)",
            line=dict(color="#EF4444", width=2.5)
        ))

    fig.update_layout(
        title="5-Channel Hybrid Power Distribution Over Time (kW)",
        xaxis_title="Mission Time (min)",
        yaxis_title="Power (kW)",
        hovermode="x unified",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
    )
    apply_dark_theme(fig)

    st.plotly_chart(fig, use_container_width=True)
