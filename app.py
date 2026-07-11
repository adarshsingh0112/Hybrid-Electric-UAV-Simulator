import streamlit as st
import pandas as pd
from pathlib import Path

st.set_page_config(
    page_title="Hybrid Electric UAV Dashboard",
    page_icon="🛩️",
    layout="wide"
)

st.title("🛩️ Hybrid Electric UAV Simulator")
st.markdown("### Mission Telemetry Dashboard")

log_file = Path("logs") / "mission_log.csv"

if not log_file.exists():
    st.warning("Mission log not found.")
    st.info("Run the simulator first:")
    st.code("python src/main.py")
    st.stop()

df = pd.read_csv(log_file)

st.success("Mission log loaded successfully.")

col1, col2, col3 = st.columns(3)

col1.metric(
    "Mission Phases",
    len(df)
)

col2.metric(
    "Final Battery SoC",
    f"{df.iloc[-1]['Battery SoC (%)']:.1f}%"
)

col3.metric(
    "Fuel Remaining",
    f"{df.iloc[-1]['Fuel Remaining (kg)']:.2f} kg"
)

st.divider()

st.subheader("Mission Data")

st.dataframe(df, use_container_width=True)

st.divider()

st.subheader("Battery SoC")

st.line_chart(
    df.set_index("Mission Time (min)")["Battery SoC (%)"]
)

st.subheader("Fuel Remaining")

st.line_chart(
    df.set_index("Mission Time (min)")["Fuel Remaining (kg)"]
)

st.subheader("Engine vs Battery Power")

power_df = df.set_index("Mission Time (min)")[
    ["Engine Power (kW)", "Battery Power (kW)"]
]

st.line_chart(power_df)

st.success("Dashboard Loaded Successfully!")