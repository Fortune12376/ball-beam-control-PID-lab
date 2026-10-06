import math
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st

from ball_beam.config import PIDConfig, PlantConfig, SensorConfig
from ball_beam.simulation import run_simulation

st.set_page_config(page_title="Ball & Beam PID Lab", page_icon="⚙️", layout="wide")
st.title("⚙️ Ball & Beam PID Lab")
st.caption("Nonlinear physics • noisy sensor • Kalman state estimation • PID control")

with st.sidebar:
    st.header("Experiment")
    beam_length = st.slider("Beam length (cm)", 20.0, 80.0, 45.0, 1.0)
    initial = st.slider("Initial ball position (cm)", 0.0, beam_length, min(8.0, beam_length), 0.5)
    target = st.slider("Target position (cm)", 0.0, beam_length, min(30.0, beam_length), 0.5)
    initial_angle = st.slider("Initial beam angle (°)", -20.0, 20.0, 0.0, 1.0)
    duration = st.slider("Duration (s)", 2.0, 20.0, 10.0, 1.0)
    dt = st.select_slider("Control timestep (s)", options=[0.005, 0.01, 0.02, 0.05], value=0.01)

    st.header("PID gains")
    kp = st.number_input("Kp", min_value=0.0, value=4.10, step=0.10, format="%.3f")
    ki = st.number_input("Ki", min_value=0.0, value=1.50, step=0.05, format="%.3f")
    kd = st.number_input("Kd", min_value=0.0, value=1.46, step=0.05, format="%.3f")

    st.header("Plant & sensor")
    friction = st.slider("Viscous friction (1/s)", 0.0, 1.5, 0.35, 0.05)
    noise = st.slider("Sensor noise σ (cm)", 0.0, 2.0, 0.4, 0.05)
    samples = st.slider("Sensor samples / control step", 1, 20, 4, 1)
    restitution = st.slider("Wall restitution", 0.0, 1.0, 0.0, 0.05)
    seed = st.number_input("Random seed", min_value=0, value=42, step=1)

plant = PlantConfig(
    beam_length_cm=beam_length,
    friction_s_inv=friction,
    restitution=restitution,
)
sensor = SensorConfig(noise_sigma_cm=noise, samples_per_control_step=samples)
pid = PIDConfig(kp=kp, ki=ki, kd=kd)

result = run_simulation(
    target_cm=target,
    initial_position_cm=initial,
    initial_angle_deg=initial_angle,
    duration_s=duration,
    dt=dt,
    seed=int(seed),
    plant_config=plant,
    sensor_config=sensor,
    pid_config=pid,
)

m = result.metrics
cols = st.columns(5)
cols[0].metric("RMS error", f"{m.rms_error:.2f} cm")
cols[1].metric("Overshoot", f"{m.overshoot_cm:.2f} cm")
cols[2].metric("Settling time", "—" if m.settling_time_s is None else f"{m.settling_time_s:.2f} s")
cols[3].metric("Steady-state error", f"{m.steady_state_error_cm:.2f} cm")
cols[4].metric("ITAE", f"{m.itae:.1f}")

left, right = st.columns([2, 1])
with left:
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=result.time, y=result.true_position, name="True position", line=dict(width=3)))
    fig.add_trace(go.Scatter(x=result.time, y=result.estimated_position, name="Kalman estimate"))
    fig.add_trace(go.Scatter(x=result.time, y=result.measured_position, name="Sensor average", opacity=0.35))
    fig.add_trace(go.Scatter(x=result.time, y=result.target, name="Target", line=dict(dash="dash")))
    fig.update_layout(title="Position tracking", xaxis_title="Time (s)", yaxis_title="Position (cm)", height=420)
    st.plotly_chart(fig, use_container_width=True)

with right:
    # Static geometry snapshot at the final state.
    angle = math.radians(result.beam_angle[-1])
    end_x = beam_length * math.cos(angle)
    end_y = beam_length * math.sin(angle)
    ball_x = result.true_position[-1] * math.cos(angle)
    ball_y = result.true_position[-1] * math.sin(angle)
    target_x = target * math.cos(angle)
    target_y = target * math.sin(angle)
    beam = go.Figure()
    beam.add_trace(go.Scatter(x=[0, end_x], y=[0, end_y], mode="lines", line=dict(width=8), name="Beam"))
    beam.add_trace(go.Scatter(x=[ball_x], y=[ball_y], mode="markers", marker=dict(size=18), name="Ball"))
    beam.add_trace(go.Scatter(x=[target_x], y=[target_y], mode="markers", marker=dict(size=15, symbol="x"), name="Target"))
    beam.update_xaxes(range=[-5, beam_length + 5], title="cm")
    beam.update_yaxes(range=[-beam_length / 2, beam_length / 2], scaleanchor="x", scaleratio=1)
    beam.update_layout(title="Final physical state", height=420, showlegend=True)
    st.plotly_chart(beam, use_container_width=True)

c1, c2 = st.columns(2)
with c1:
    fig2 = go.Figure()
    fig2.add_trace(go.Scatter(x=result.time, y=result.true_velocity, name="True velocity"))
    fig2.add_trace(go.Scatter(x=result.time, y=result.estimated_velocity, name="Kalman velocity", line=dict(dash="dash")))
    fig2.update_layout(title="Velocity estimation", xaxis_title="Time (s)", yaxis_title="Velocity (cm/s)", height=360)
    st.plotly_chart(fig2, use_container_width=True)
with c2:
    fig3 = go.Figure()
    fig3.add_trace(go.Scatter(x=result.time, y=result.beam_angle, name="Actual beam angle"))
    fig3.add_trace(go.Scatter(x=result.time, y=result.commanded_angle, name="Commanded angle", opacity=0.7))
    fig3.update_layout(title="Actuator behaviour", xaxis_title="Time (s)", yaxis_title="Angle (°)", height=360)
    st.plotly_chart(fig3, use_container_width=True)

st.subheader("Animated physical simulation")
frame_stride = max(1, len(result.time) // 150)
frame_indices = list(range(0, len(result.time), frame_stride))
if frame_indices[-1] != len(result.time) - 1:
    frame_indices.append(len(result.time) - 1)


def geometry(i):
    a = math.radians(result.beam_angle[i])
    return (
        beam_length * math.cos(a), beam_length * math.sin(a),
        result.true_position[i] * math.cos(a), result.true_position[i] * math.sin(a),
        target * math.cos(a), target * math.sin(a),
    )

ex, ey, bx, by, tx, ty = geometry(frame_indices[0])
anim = go.Figure(
    data=[
        go.Scatter(x=[0, ex], y=[0, ey], mode="lines", line=dict(width=8), name="Beam"),
        go.Scatter(x=[bx], y=[by], mode="markers", marker=dict(size=18), name="Ball"),
        go.Scatter(x=[tx], y=[ty], mode="markers", marker=dict(size=15, symbol="x"), name="Target"),
    ],
    frames=[
        go.Frame(
            name=str(i),
            data=[
                go.Scatter(x=[0, geometry(i)[0]], y=[0, geometry(i)[1]]),
                go.Scatter(x=[geometry(i)[2]], y=[geometry(i)[3]]),
                go.Scatter(x=[geometry(i)[4]], y=[geometry(i)[5]]),
            ],
        ) for i in frame_indices
    ],
)
anim.update_layout(
    height=440,
    xaxis=dict(range=[-5, beam_length + 5], title="cm"),
    yaxis=dict(range=[-beam_length / 2, beam_length / 2], scaleanchor="x", scaleratio=1),
    updatemenus=[{
        "type": "buttons",
        "buttons": [
            {"label": "▶ Play", "method": "animate", "args": [None, {"frame": {"duration": 35, "redraw": True}, "fromcurrent": True}]},
            {"label": "⏸ Pause", "method": "animate", "args": [[None], {"frame": {"duration": 0}, "mode": "immediate"}]},
        ],
    }],
    sliders=[{
        "steps": [{"method": "animate", "label": f"{result.time[i]:.1f}", "args": [[str(i)], {"mode": "immediate", "frame": {"duration": 0, "redraw": True}}]} for i in frame_indices[::max(1, len(frame_indices)//20)]],
        "currentvalue": {"prefix": "Time: "},
    }],
)
st.plotly_chart(anim, use_container_width=True)

with st.expander("Raw simulation data"):
    df = pd.DataFrame({
        "time_s": result.time,
        "true_position_cm": result.true_position,
        "estimated_position_cm": result.estimated_position,
        "measured_position_cm": result.measured_position,
        "true_velocity_cm_s": result.true_velocity,
        "estimated_velocity_cm_s": result.estimated_velocity,
        "beam_angle_deg": result.beam_angle,
        "command_deg": result.commanded_angle,
    })
    st.dataframe(df, use_container_width=True, height=320)
    st.download_button("Download CSV", df.to_csv(index=False), "ball_beam_run.csv", "text/csv")

st.info("The simulation is educational: it is a nonlinear software model, not a hardware safety controller.")
