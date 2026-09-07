import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import streamlit as st
from uav_dashboard.styles import apply_dark_theme


def render_phase_timeline(df: pd.DataFrame):
    """Renders interactive Gantt bar chart representing mission phase timeline transitions."""
    st.markdown("### ⏱️ Mission Phase Timeline")

    if "Phase" not in df.columns or "Mission Time (min)" not in df.columns:
        st.warning("Phase timeline data missing from telemetry.")
        return

    # Extract phase start and end timestamps
    phase_summary = []
    current_phase = None
    start_t = 0.0

    for idx, row in df.iterrows():
        p = row["Phase"]
        t = row["Mission Time (min)"]

        if p != current_phase:
            if current_phase is not None:
                phase_summary.append({
                    "Phase": current_phase,
                    "Start (min)": round(start_t, 2),
                    "End (min)": round(t, 2),
                    "Duration (min)": round(t - start_t, 2)
                })
            current_phase = p
            start_t = t

    # Append final phase
    if current_phase is not None and len(df) > 0:
        end_t = df["Mission Time (min)"].iloc[-1]
        phase_summary.append({
            "Phase": current_phase,
            "Start (min)": round(start_t, 2),
            "End (min)": round(end_t, 2),
            "Duration (min)": round(end_t - start_t, 2)
        })

    phase_df = pd.DataFrame(phase_summary)

    # Plotly Timeline Bar Chart
    fig = go.Figure()

    colors = {
        "Takeoff": "#38BDF8",
        "Climb": "#F59E0B",
        "Cruise": "#34D399",
        "Loiter": "#C084FC",
        "Descent": "#E879F9",
        "Landing": "#F87171"
    }

    for idx, row in phase_df.iterrows():
        p_name = row["Phase"]
        color = colors.get(p_name, "#94A3B8")

        fig.add_trace(go.Bar(
            y=[p_name],
            x=[row["Duration (min)"]],
            base=[row["Start (min)"]],
            orientation="h",
            name=p_name,
            marker_color=color,
            hovertemplate=(
                f"<b>Phase: {p_name}</b><br>" +
                f"Start: {row['Start (min)']} min<br>" +
                f"End: {row['End (min)']} min<br>" +
                f"Duration: {row['Duration (min)']} min<extra></extra>"
            )
        ))

    fig.update_layout(
        title="Mission Phase Gantt Timeline (Minutes)",
        xaxis_title="Mission Time (min)",
        yaxis_title="Phase Name",
        showlegend=False,
        height=320
    )
    apply_dark_theme(fig)

    st.plotly_chart(fig, use_container_width=True)

    # Phase Breakdown Summary Table
    with st.expander("📋 View Mission Phase Duration Breakdown Table"):
        st.dataframe(phase_df, use_container_width=True)
