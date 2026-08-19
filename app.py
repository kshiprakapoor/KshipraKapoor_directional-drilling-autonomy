from __future__ import annotations

import io
import time
import zipfile
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st

from agent_orchestrator import AutonomousSuperintendent
from control_theory import TrajectoryMPC
from physics_engine import ObstacleBox, WellboreDynamics


st.set_page_config(
    page_title="Directional Drilling Autonomous Avoidance",
    page_icon="⛏️",
    layout="wide",
)


# --- INDUSTRIAL CONTROL ROOM PRESENTATION LAYER ---
st.markdown(
    """
    <style>
        .stApp { background: #0b0f14; color: #e5edf5; }
        [data-testid="stSidebar"] { background: #101720; }
        .terminal {
            background: #05080b;
            border: 1px solid #263340;
            border-radius: 8px;
            padding: 14px;
            height: 270px;
            overflow-y: auto;
            font-family: "SFMono-Regular", Consolas, "Liberation Mono", monospace;
            font-size: 0.82rem;
            line-height: 1.45;
            color: #c7f9cc;
        }
        .engineering-note {
            border-left: 3px solid #5e81ac;
            padding-left: 12px;
            color: #b9c6d3;
        }
    </style>
    """,
    unsafe_allow_html=True,
)


PROJECT_FILES = [
    "app.py",
    "physics_engine.py",
    "control_theory.py",
    "agent_orchestrator.py",
    "requirements.txt",
    "README.md",
    "DEPLOYMENT.md",
    ".gitignore",
    ".streamlit/config.toml",
]


def build_project_archive() -> bytes:
    archive_buffer = io.BytesIO()
    project_root = Path(__file__).resolve().parent
    with zipfile.ZipFile(archive_buffer, mode="w", compression=zipfile.ZIP_DEFLATED) as archive:
        for filename in PROJECT_FILES:
            source_path = project_root / filename
            if source_path.exists():
                archive.write(source_path, arcname=filename)
    archive_buffer.seek(0)
    return archive_buffer.getvalue()


def nominal_path(
    start_hd: float,
    start_tvd: float,
    target_hd: float,
    target_tvd: float,
) -> list[tuple[float, float]]:
    tvd_values = np.linspace(start_tvd, target_tvd, 80)
    hd_values = np.interp(tvd_values, [start_tvd, target_tvd], [start_hd, target_hd])
    return list(zip(hd_values, tvd_values))


def initialize_simulation(target_hd_ft: float, target_tvd_ft: float) -> None:
    start_tvd_ft = 5000.0
    start_hd_ft = 0.0
    initial_inclination_deg = float(
        np.degrees(np.arctan2(target_hd_ft - start_hd_ft, target_tvd_ft - start_tvd_ft))
    )

    obstacle = ObstacleBox()
    st.session_state.dynamics = WellboreDynamics(
        start_tvd_ft=start_tvd_ft,
        start_hd_ft=start_hd_ft,
        initial_inclination_deg=initial_inclination_deg,
        obstacle=obstacle,
    )
    st.session_state.controller = TrajectoryMPC(
        max_dls_deg_per_100ft=4.0,
        prediction_horizon=6,
        md_step_ft=50.0,
    )
    st.session_state.superintendent = AutonomousSuperintendent(
        obstacle=obstacle,
        trigger_distance_ft=400.0,
        detour_dls_deg_per_100ft=3.5,
        clearance_ft=90.0,
    )
    st.session_state.nominal_target_path = nominal_path(
        start_hd_ft,
        start_tvd_ft,
        target_hd_ft,
        target_tvd_ft,
    )
    st.session_state.active_target_path = list(st.session_state.nominal_target_path)
    st.session_state.running = False
    st.session_state.completed = False
    st.session_state.last_mpc_force_kn = 0.0
    st.session_state.last_predicted_dls = 0.0
    st.session_state.interface_log = [
        "[SYSTEM] Simulation initialized at TVD 5,000 ft.",
        "[SYSTEM] Local trajectory MPC armed; DLS hard limit = 4.0°/100 ft.",
    ]


def render_terminal(events: list[str]) -> None:
    visible = events[-18:]
    terminal_html = "<br>".join(
        event.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        for event in visible
    )
    st.markdown(f'<div class="terminal">{terminal_html}</div>', unsafe_allow_html=True)


def trajectory_figure(
    dynamics: WellboreDynamics,
    nominal_target_path: list[tuple[float, float]],
    active_target_path: list[tuple[float, float]],
    target_hd_ft: float,
    target_tvd_ft: float,
    override_active: bool,
) -> go.Figure:
    history = pd.DataFrame(dynamics.history)
    nominal = np.asarray(nominal_target_path)
    active = np.asarray(active_target_path)
    obstacle = dynamics.obstacle

    fig = go.Figure()

    # --- GEOLOGICAL EXCLUSION VOLUME: section-view projection of the unmapped hazard ---
    fig.add_shape(
        type="rect",
        x0=obstacle.hd_min_ft,
        x1=obstacle.hd_max_ft,
        y0=obstacle.tvd_min_ft,
        y1=obstacle.tvd_max_ft,
        fillcolor="rgba(220, 53, 69, 0.34)",
        line=dict(color="rgba(255, 99, 110, 0.95)", width=2),
        layer="below",
    )
    fig.add_annotation(
        x=(obstacle.hd_min_ft + obstacle.hd_max_ft) / 2,
        y=(obstacle.tvd_min_ft + obstacle.tvd_max_ft) / 2,
        text="UNMAPPED HAZARD",
        showarrow=False,
        font=dict(size=12),
    )

    fig.add_trace(
        go.Scatter(
            x=nominal[:, 0],
            y=nominal[:, 1],
            mode="lines",
            name="Original plan",
            line=dict(color="#52d273", dash="dash", width=2),
        )
    )

    if override_active:
        fig.add_trace(
            go.Scatter(
                x=active[:, 0],
                y=active[:, 1],
                mode="lines",
                name="Agent detour setpoints",
                line=dict(color="#55c2ff", dash="dot", width=2),
            )
        )

    fig.add_trace(
        go.Scatter(
            x=history["hd_ft"],
            y=history["tvd_ft"],
            mode="lines+markers",
            name="Drilled trajectory",
            line=dict(color="#ff5c5c", width=4),
            marker=dict(color="#ff5c5c", size=5),
        )
    )

    fig.add_trace(
        go.Scatter(
            x=[history["hd_ft"].iloc[-1]],
            y=[history["tvd_ft"].iloc[-1]],
            mode="markers",
            name="Bit",
            marker=dict(color="#ffffff", size=13, symbol="diamond"),
        )
    )

    fig.add_trace(
        go.Scatter(
            x=[target_hd_ft],
            y=[target_tvd_ft],
            mode="markers",
            name="Target",
            marker=dict(color="#ffd166", size=14, symbol="star"),
        )
    )

    fig.update_layout(
        height=600,
        margin=dict(l=20, r=20, t=55, b=30),
        title="Directional Section — Closed-Loop Trajectory Tracking",
        xaxis_title="Horizontal Displacement, HD (ft)",
        yaxis_title="True Vertical Depth, TVD (ft)",
        xaxis=dict(range=[0, 3000], gridcolor="rgba(255,255,255,0.08)"),
        yaxis=dict(range=[8200, 5000], gridcolor="rgba(255,255,255,0.08)"),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="left", x=0),
        paper_bgcolor="#0b0f14",
        plot_bgcolor="#0b0f14",
        font=dict(color="#dfe7ef"),
    )
    return fig


# --- OPERATOR SETPOINTS AND SUPERVISORY MODE SELECTION ---
st.sidebar.header("Drilling Controls")
wob_klbf = st.sidebar.slider("Weight on Bit (klbf)", 8.0, 45.0, 24.0, 1.0)
target_hd_ft = st.sidebar.number_input(
    "Target Horizontal Displacement (ft)",
    min_value=1400.0,
    max_value=2800.0,
    value=1500.0,
    step=50.0,
)
target_tvd_ft = st.sidebar.number_input(
    "Target TVD (ft)",
    min_value=7000.0,
    max_value=8200.0,
    value=8000.0,
    step=50.0,
)
mode = st.sidebar.radio(
    "Supervisory Mode",
    ["Agentic Override Mode", "Legacy Automation Mode"],
    index=0,
)

if "dynamics" not in st.session_state:
    initialize_simulation(target_hd_ft, target_tvd_ft)

control_col_a, control_col_b = st.sidebar.columns(2)
if control_col_a.button("Start / Resume", use_container_width=True):
    st.session_state.running = True
if control_col_b.button("Reset", use_container_width=True):
    initialize_simulation(target_hd_ft, target_tvd_ft)
    st.rerun()

st.sidebar.caption(
    "Interview-scale reduced-order simulation. Steering force and vibration models are illustrative, "
    "not Halliburton proprietary models."
)
    
st.title(
    "Directional Drilling "
    "Obstacle-Avoidance Simulation"
)

st.markdown(
    """
    **Developed by Kshipra Kapoor, PhD**  
    *Electrical & Computer Engineering | Automation | AI-Enabled Engineering Systems*
    """
)

st.caption(
    "Constrained trajectory MPC + supervisory autonomous "
    "hazard avoidance + structural-health telemetry"
)
archive_bytes = build_project_archive()
st.download_button(
    "Download Complete Project Archive (.zip)",
    data=archive_bytes,
    file_name="directional_drilling_obstacle_avoidance.zip",
    mime="application/zip",
    use_container_width=False,
)

dynamics: WellboreDynamics = st.session_state.dynamics
controller: TrajectoryMPC = st.session_state.controller
superintendent: AutonomousSuperintendent = st.session_state.superintendent

telemetry = dynamics.snapshot()
agentic_mode = mode == "Agentic Override Mode"
decision = superintendent.evaluate(
    telemetry=telemetry,
    nominal_target_path=st.session_state.nominal_target_path,
    final_target=(target_hd_ft, target_tvd_ft),
    agentic_mode=agentic_mode,
)
st.session_state.active_target_path = decision.target_path
st.session_state.interface_log.extend(decision.events)

# --- CLOSED-LOOP EXECUTION STEP: supervisory path → constrained MPC → reduced-order BHA plant ---
if st.session_state.running and not st.session_state.completed:
    mpc_solution = controller.solve(telemetry, st.session_state.active_target_path)
    st.session_state.last_mpc_force_kn = mpc_solution.steering_pad_force_kn
    st.session_state.last_predicted_dls = mpc_solution.predicted_dls_deg_per_100ft

    telemetry = dynamics.step(
        steering_pad_force_kn=mpc_solution.steering_pad_force_kn,
        wob_klbf=wob_klbf,
        md_step_ft=50.0,
    )

    if telemetry["collision_active"]:
        st.session_state.interface_log.append(
            f"[ALARM] Obstacle penetration detected: vibration={telemetry['vibration_g']:.2f} g; "
            f"tool health={telemetry['tool_health_pct']:.1f}%."
        )

    target_distance_ft = float(
        np.hypot(target_hd_ft - telemetry["hd_ft"], target_tvd_ft - telemetry["tvd_ft"])
    )
    if (
        target_distance_ft < 85.0
        or telemetry["tvd_ft"] >= target_tvd_ft
        or telemetry["tool_health_pct"] < 3.0
    ):
        st.session_state.running = False
        st.session_state.completed = True
        if telemetry["tool_health_pct"] < 3.0:
            st.session_state.interface_log.append(
                "[TRIP] Structural-health threshold exhausted; drilling halted to protect the BHA."
            )
        else:
            st.session_state.interface_log.append(
                "[COMPLETE] Target interval reached; closed-loop sequence terminated."
            )

    time.sleep(0.3)
    st.rerun()

telemetry = dynamics.snapshot()
target_distance_ft = float(
    np.hypot(target_hd_ft - telemetry["hd_ft"], target_tvd_ft - telemetry["tvd_ft"])
)

left_col, right_col = st.columns([1.18, 1.0], gap="large")

with left_col:
    st.subheader("Supervisory Engineering Audit")
    render_terminal(st.session_state.interface_log)

with right_col:
    st.subheader("Live Telemetry")
    metric_a, metric_b = st.columns(2)
    metric_a.metric("Bit TVD", f"{telemetry['tvd_ft']:,.0f} ft")
    metric_b.metric("Bit HD", f"{telemetry['hd_ft']:,.0f} ft")

    metric_c, metric_d = st.columns(2)
    metric_c.metric("Target Deviation", f"{target_distance_ft:,.0f} ft")
    metric_d.metric("Inclination", f"{telemetry['inclination_deg']:.1f}°")

    metric_e, metric_f = st.columns(2)
    metric_e.metric("Mechanical Vibration", f"{telemetry['vibration_g']:.2f} g")
    metric_f.metric("Tool Health", f"{telemetry['tool_health_pct']:.1f}%")

    metric_g, metric_h = st.columns(2)
    metric_g.metric("ROP", f"{telemetry['rop_ft_hr']:.1f} ft/hr")
    metric_h.metric("Actual DLS", f"{telemetry['dls_deg_per_100ft']:.2f}°/100 ft")

    st.markdown(
        f"""
        <div class="engineering-note">
        <b>Controller:</b> constrained finite-horizon MPC<br>
        <b>Steering command:</b> {st.session_state.last_mpc_force_kn:+.2f} kN<br>
        <b>DLS limit:</b> 4.00°/100 ft<br>
        <b>Hazard stand-off:</b> {telemetry['distance_to_obstacle_ft']:.0f} ft<br>
        <b>Supervisory override:</b> {"ACTIVE" if superintendent.override_active and agentic_mode else "STANDBY"}
        </div>
        """,
        unsafe_allow_html=True,
    )

st.plotly_chart(
    trajectory_figure(
        dynamics,
        st.session_state.nominal_target_path,
        st.session_state.active_target_path,
        target_hd_ft,
        target_tvd_ft,
        superintendent.override_active and agentic_mode,
    ),
    use_container_width=True,
)

if mode == "Legacy Automation Mode":
    st.warning(
        "Legacy mode keeps the local MPC on the original trajectory. It has no supervisory obstacle re-planning layer."
    )
else:
    st.success(
        "Agentic mode monitors the geometric look-ahead zone and can replace trajectory setpoints while leaving low-level steering to the constrained MPC."
    )
st.markdown("---")

st.caption(
    "Developed by Kshipra Kapoor, PhD | "
    "Electrical & Computer Engineering | "
    "Independent engineering R&D demonstration"
)
