import pandas as pd
import streamlit as st
from pathlib import Path

st.set_page_config(
    page_title="Hybrid Electric UAV Simulator",
    layout="wide"
)

st.title("🛩️ Hybrid Electric UAV Digital Twin")

log_file = Path("logs/mission_log.csv")

if not log_file.exists():

    st.warning("Mission log not found.\nRun the simulator first.")

    st.stop()

df = pd.read_csv(log_file)

# -------------------------
# Latest Status
# -------------------------

latest = df.iloc[-1]

st.header("Current UAV Status")

col1, col2, col3 = st.columns(3)

with col1:
    st.metric("Battery SoC", f"{latest['Battery SoC (%)']} %")

with col2:
    st.metric("Fuel Remaining", f"{latest['Fuel Remaining (kg)']} kg")

with col3:
    st.metric("Motor RPM", int(latest["Motor RPM"]))

# -------------------------
# Charts
# -------------------------

st.header("Mission Telemetry")

st.line_chart(
    df.set_index("Mission Time (min)")["Battery SoC (%)"]
)

st.line_chart(
    df.set_index("Mission Time (min)")["Fuel Remaining (kg)"]
)

st.line_chart(
    df.set_index("Mission Time (min)")["Engine Power (kW)"]
)

st.line_chart(
    df.set_index("Mission Time (min)")["Battery Power (kW)"]
)

st.dataframe(df)