"""
uav_dashboard/styles.py
Centralized CSS styling and dark aerospace Plotly theme module for Hybrid UAV Simulator.
"""

import streamlit as st
import plotly.graph_objects as go

# Dark Aerospace Color System
BG_DARK = "#0B0F19"
SURFACE_DARK = "rgba(18, 26, 43, 0.85)"
BORDER_CYAN = "rgba(56, 189, 248, 0.2)"
ACCENT_CYAN = "#38BDF8"
ACCENT_BLUE = "#0EA5E9"

CSS_STYLES = """
<style>
/* Base App & Layout Background */
html, body, .stApp, [data-testid="stAppViewContainer"], .main, [data-testid="stHeader"] {
    background-color: #0B0F19 !important;
    background-image: 
        radial-gradient(circle at 50% 0%, rgba(14, 165, 233, 0.15) 0%, transparent 65%),
        radial-gradient(circle at 85% 30%, rgba(16, 185, 129, 0.08) 0%, transparent 45%),
        linear-gradient(rgba(56, 189, 248, 0.04) 1px, transparent 1px),
        linear-gradient(90deg, rgba(56, 189, 248, 0.04) 1px, transparent 1px) !important;
    background-size: 100% 100%, 100% 100%, 32px 32px, 32px 32px !important;
    background-position: 0 0, 0 0, -1px -1px, -1px -1px !important;
    color: #F8FAFC !important;
    font-family: 'Inter', -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif !important;
}

/* Sidebar Custom Styling */
[data-testid="stSidebar"], section[data-testid="stSidebar"] {
    background-color: #060911 !important;
    border-right: 1px solid rgba(56, 189, 248, 0.18) !important;
    box-shadow: 4px 0 20px rgba(0, 0, 0, 0.5) !important;
}

[data-testid="stSidebar"] [data-testid="stMarkdownContainer"] h1,
[data-testid="stSidebar"] [data-testid="stMarkdownContainer"] h2,
[data-testid="stSidebar"] [data-testid="stMarkdownContainer"] h3 {
    color: #38BDF8 !important;
    font-weight: 600 !important;
    letter-spacing: 0.5px !important;
}

/* Hero / Header Section */
.uav-hero-container {
    background: linear-gradient(135deg, rgba(18, 26, 43, 0.9) 0%, rgba(11, 15, 25, 0.95) 100%) !important;
    border: 1px solid rgba(56, 189, 248, 0.25) !important;
    border-radius: 12px !important;
    padding: 22px 28px !important;
    margin-bottom: 24px !important;
    box-shadow: 0 8px 24px -6px rgba(0, 0, 0, 0.6), 0 0 20px -3px rgba(14, 165, 233, 0.2) !important;
    position: relative !important;
    overflow: hidden !important;
}

.uav-hero-container::before {
    content: "" !important;
    position: absolute !important;
    top: 0 !important;
    left: 0 !important;
    right: 0 !important;
    height: 3px !important;
    background: linear-gradient(90deg, #38BDF8 0%, #0EA5E9 50%, transparent 100%) !important;
}

.uav-hero-title {
    font-size: 2.1rem !important;
    font-weight: 800 !important;
    color: #F8FAFC !important;
    letter-spacing: -0.5px !important;
    margin: 0 0 8px 0 !important;
    display: flex !important;
    align-items: center !important;
    gap: 12px !important;
    text-shadow: 0 2px 10px rgba(0, 0, 0, 0.5) !important;
}

.uav-hero-subtitle {
    font-size: 0.95rem !important;
    color: #94A3B8 !important;
    margin: 0 !important;
    display: flex !important;
    align-items: center !important;
    gap: 14px !important;
    flex-wrap: wrap !important;
}

/* Status Indicator Dot */
.status-pulse-dot {
    display: inline-block !important;
    width: 10px !important;
    height: 10px !important;
    border-radius: 50% !important;
    background-color: #10B981 !important;
    box-shadow: 0 0 10px #10B981 !important;
    animation: pulseDot 2s infinite !important;
}

@keyframes pulseDot {
    0% { opacity: 1; transform: scale(1); }
    50% { opacity: 0.35; transform: scale(0.85); }
    100% { opacity: 1; transform: scale(1); }
}

.status-badge {
    display: inline-flex !important;
    align-items: center !important;
    gap: 8px !important;
    background: rgba(16, 185, 129, 0.12) !important;
    border: 1px solid rgba(16, 185, 129, 0.4) !important;
    color: #34D399 !important;
    padding: 6px 14px !important;
    border-radius: 20px !important;
    font-size: 0.8rem !important;
    font-weight: 700 !important;
    letter-spacing: 0.6px !important;
    box-shadow: 0 0 12px rgba(16, 185, 129, 0.15) !important;
}

/* Streamlit Native Metric Overrides */
div[data-testid="stMetric"] {
    background: rgba(18, 26, 43, 0.85) !important;
    border: 1px solid rgba(56, 189, 248, 0.2) !important;
    border-radius: 12px !important;
    padding: 14px 18px !important;
    transition: all 0.2s ease-in-out !important;
    box-shadow: 0 4px 14px rgba(0, 0, 0, 0.3) !important;
}

div[data-testid="stMetric"]:hover {
    border-color: rgba(56, 189, 248, 0.5) !important;
    transform: translateY(-2px) !important;
    box-shadow: 0 8px 24px -4px rgba(14, 165, 233, 0.25) !important;
}

div[data-testid="stMetricLabel"] {
    color: #94A3B8 !important;
    font-weight: 700 !important;
    font-size: 0.82rem !important;
    text-transform: uppercase !important;
    letter-spacing: 0.6px !important;
}

div[data-testid="stMetricValue"] {
    color: #F8FAFC !important;
    font-weight: 800 !important;
    font-size: 1.55rem !important;
}

div[data-testid="stMetricDelta"] {
    font-weight: 600 !important;
}

/* Modern Tab Navigation Overrides */
.stTabs [data-baseweb="tab-list"] {
    gap: 10px !important;
    background: rgba(11, 15, 25, 0.8) !important;
    padding: 8px !important;
    border-radius: 12px !important;
    border: 1px solid rgba(56, 189, 248, 0.2) !important;
    margin-bottom: 20px !important;
}

.stTabs [data-baseweb="tab"] {
    border-radius: 8px !important;
    padding: 10px 20px !important;
    font-weight: 700 !important;
    font-size: 0.92rem !important;
    color: #94A3B8 !important;
    border: none !important;
    background: transparent !important;
    transition: all 0.2s ease-in-out !important;
}

.stTabs [data-baseweb="tab"]:hover {
    color: #F8FAFC !important;
    background: rgba(56, 189, 248, 0.1) !important;
}

.stTabs [aria-selected="true"] {
    background: rgba(14, 165, 233, 0.25) !important;
    color: #38BDF8 !important;
    border-bottom: 3px solid #38BDF8 !important;
    box-shadow: 0 4px 14px rgba(14, 165, 233, 0.3) !important;
}

/* Hardware Specs Panel */
.uav-specs-box {
    background: rgba(15, 23, 42, 0.9) !important;
    border: 1px solid rgba(56, 189, 248, 0.2) !important;
    border-radius: 10px !important;
    padding: 14px 16px !important;
    font-size: 0.85rem !important;
    color: #CBD5E1 !important;
    box-shadow: inset 0 0 10px rgba(0, 0, 0, 0.5) !important;
}

.uav-specs-grid {
    display: grid !important;
    grid-template-columns: repeat(2, 1fr) !important;
    gap: 10px 14px !important;
    margin-top: 10px !important;
}

.uav-spec-item {
    display: flex !important;
    flex-direction: column !important;
}

.uav-spec-label {
    font-size: 0.72rem !important;
    color: #94A3B8 !important;
    text-transform: uppercase !important;
    letter-spacing: 0.5px !important;
    font-weight: 600 !important;
}

.uav-spec-val {
    font-weight: 700 !important;
    color: #38BDF8 !important;
    font-size: 0.95rem !important;
}

/* Divider Styling */
hr {
    border-color: rgba(56, 189, 248, 0.18) !important;
}

/* Alert HUD Panels */
.uav-status-hud {
    border-radius: 12px !important;
    padding: 18px 24px !important;
    margin-bottom: 24px !important;
    border: 1px solid !important;
    display: flex !important;
    align-items: center !important;
    justify-content: space-between !important;
    box-shadow: 0 6px 20px rgba(0, 0, 0, 0.4) !important;
}

.uav-status-hud.success {
    background: rgba(16, 185, 129, 0.12) !important;
    border-color: rgba(16, 185, 129, 0.4) !important;
    color: #34D399 !important;
}

.uav-status-hud.warning {
    background: rgba(245, 158, 11, 0.12) !important;
    border-color: rgba(245, 158, 11, 0.4) !important;
    color: #FBBF24 !important;
}

.uav-status-hud.error {
    background: rgba(239, 68, 68, 0.12) !important;
    border-color: rgba(239, 68, 68, 0.4) !important;
    color: #F87171 !important;
}

/* Plotly Element Styling */
.js-plotly-plot {
    border-radius: 12px !important;
    border: 1px solid rgba(56, 189, 248, 0.15) !important;
    background: rgba(18, 26, 43, 0.6) !important;
    padding: 8px !important;
    box-shadow: 0 4px 16px rgba(0, 0, 0, 0.4) !important;
    margin-bottom: 12px !important;
}
</style>
"""


def inject_custom_css():
    """Injects custom aerospace dark theme CSS into Streamlit page."""
    st.markdown(CSS_STYLES, unsafe_allow_html=True)


def apply_dark_theme(fig: go.Figure) -> go.Figure:
    """
    Applies consistent dark aerospace engineering styling to a Plotly figure.
    Preserves trace colors while setting dark backgrounds, custom gridlines, and typography.
    """
    fig.update_layout(
        paper_bgcolor="rgba(0, 0, 0, 0)",
        plot_bgcolor="rgba(15, 23, 42, 0.45)",
        font=dict(
            family="Inter, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif",
            color="#94A3B8",
            size=12
        ),
        title=dict(
            font=dict(color="#F8FAFC", size=14, family="Inter, sans-serif"),
            x=0.01,
            xanchor="left"
        ),
        xaxis=dict(
            gridcolor="rgba(255, 255, 255, 0.06)",
            zerolinecolor="rgba(255, 255, 255, 0.1)",
            linecolor="rgba(56, 189, 248, 0.15)",
            tickfont=dict(color="#94A3B8")
        ),
        yaxis=dict(
            gridcolor="rgba(255, 255, 255, 0.06)",
            zerolinecolor="rgba(255, 255, 255, 0.1)",
            linecolor="rgba(56, 189, 248, 0.15)",
            tickfont=dict(color="#94A3B8")
        ),
        legend=dict(
            font=dict(color="#CBD5E1", size=11),
            bgcolor="rgba(15, 23, 42, 0.6)",
            bordercolor="rgba(56, 189, 248, 0.15)",
            borderwidth=1
        ),
        margin=dict(l=40, r=20, t=40, b=40)
    )
    return fig
