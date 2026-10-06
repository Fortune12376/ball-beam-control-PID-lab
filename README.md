# Ball & Beam PID Lab

A nonlinear ball-and-beam control simulation with a PID controller, noisy sensor model, Kalman state estimation, automatic gain tuning, performance metrics, and an interactive Streamlit visualisation.

## What it demonstrates

- Nonlinear ball dynamics using
```math
\ddot{x} = \frac{5}{7}g\sin(\theta) - b\dot{x}
```
- Servo angle saturation and slew-rate limits
- Viscous friction, optional wall restitution, and simple stiction
- Gaussian sensor noise, quantisation, and sample averaging
- Two-state Kalman estimation of position and velocity
- PID control with derivative-on-measurement
- Conditional-integration anti-windup tied to actuator saturation
- Multi-scenario Twiddle / coordinate-descent tuning of Kp, Ki, and Kd independently
- IAE, ISE, ITAE, RMS error, overshoot, settling time, steady-state error, and actuator-travel metrics
- Interactive Plotly graphs and a Streamlit beam/ball animation
- Unit tests and GitHub Actions CI

## Why the derivative term uses velocity

For a fixed target \(r\), the error is \(e=r-x\), so

```math
\frac{de}{dt} = -\dot{x}
```

Instead of differentiating a noisy position measurement, this project estimates \(\dot{x}\) with the Kalman filter and applies

```math
\theta_c=K_p e + K_i\int e\,dt - K_d\hat{\dot{x}}.
```

That is derivative-on-measurement: setpoint changes do not create a derivative kick, and the derivative signal is much less noisy than a direct finite difference.

## A note on critical damping

For small beam angles:

```math
\sin(\theta) \approx \theta
```

where \(\theta\) is in radians. Because the controller output is expressed in degrees, define:

```math
C = \frac{5}{7}g\frac{\pi}{180}
```

Ignoring integral action, the approximate plant/controller pair is:

```math
\ddot{x} = C\theta - b\dot{x}
```

```math
\theta = K_p(r-x) - K_d\dot{x}
```

The closed-loop characteristic equation becomes:

```math
s^2 + (b + CK_d)s + CK_p = 0
```

Critical damping therefore gives:

```math
(b + CK_d)^2 = 4CK_p
```

or:

```math
K_d = \frac{2\sqrt{CK_p} - b}{C}
```

This is useful as an initial guess, but not as a hard constraint. The real simulator contains nonlinear geometry, saturation, slew limits, sensor noise, filtering, integral action, and wall interactions, so the included tuner optimises all three PID gains independently.

## Project structure

```text
ball-beam-pid/
├── app.py                    # Streamlit dashboard + animation
├── run_demo.py               # Lightweight CLI simulation
├── tune.py                   # Three-parameter Twiddle tuner
├── ball_beam/
│   ├── config.py             # Dataclass configuration
│   ├── physics.py            # Nonlinear plant + sensor
│   ├── filters.py            # Position/velocity Kalman filter
│   ├── controller.py         # PID + anti-windup
│   ├── metrics.py            # Performance metrics
│   ├── simulation.py         # Closed-loop experiment runner
│   └── tuning.py             # Robust multi-scenario tuning
├── tests/
├── .github/workflows/ci.yml
├── requirements.txt
├── requirements-dev.txt
└── pyproject.toml
```

## Quick start

### 1. Clone and create a virtual environment

```bash
git clone https://github.com/Fortune12376/ball-beam-pid.git
cd ball-beam-pid
python -m venv .venv
```

Activate it:

```bash
# macOS / Linux
source .venv/bin/activate
```

```powershell
# Windows PowerShell
.venv\Scripts\Activate.ps1
```

### 2. Install dependencies

```bash
pip install -r requirements.txt
```

### 3. Launch the interactive dashboard

```bash
streamlit run app.py
```

Use the sidebar to change the target, initial state, PID gains, friction, sensor noise, sampling rate, and wall restitution. The dashboard recalculates the experiment and shows tracking, state-estimation quality, actuator behaviour, performance metrics, an animated physical view, and downloadable CSV data.

## CLI demo

```bash
python run_demo.py
```

This runs the default experiment and prints a compact set of performance metrics.

## Automatic PID tuning

```bash
python tune.py
```

The tuner uses deterministic noise seeds and several start/target/angle scenarios so it is less likely to overfit a single trajectory. The objective is dominated by ITAE and adds small penalties for actuator travel and final error.

The tuner is intentionally kept separate from the Streamlit UI because full optimisation is much more computationally expensive than running one simulation.

## Testing

Install the development dependencies:

```bash
pip install -r requirements-dev.txt
```

Run the tests:

```bash
pytest -q
```

Run the linter:

```bash
ruff check .
```

GitHub Actions runs the same lint and test checks on pushes and pull requests.

## Model assumptions

Important assumptions include:

- The ball is treated as a solid sphere rolling without slipping, giving the acceleration factor:

```math
\frac{5}{7}
```

- Beam angle directly determines the gravity component along the beam.
- Friction is represented by a linear viscous term.
- The servo has a hard angle limit and constant maximum angular speed.
- The Kalman estimator uses a simplified local state-transition Jacobian.
- No beam flex, backlash, motor dead-band, sensor latency, or ball/beam contact loss is modelled.

Those are useful next steps if the project is later connected to real hardware.

## Possible extensions

Good follow-on experiments include an LQR controller, state-space linearisation around the centre point, MPC, servo dead-band/backlash, sensor latency, system identification from real logs, comparison of PID tuning methods, or a hardware mode that swaps the simulated sensor/servo interfaces for real I/O.

## Safety / scope

The code is intended for simulation. If adapted to physical hardware, add independent mechanical and electrical limits, emergency-stop behaviour, watchdogs, and hardware-specific validation rather than relying on the simulator's software limits.

## License

MIT.