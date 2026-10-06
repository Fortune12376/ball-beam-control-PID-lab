from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class PerformanceMetrics:
    iae: float
    ise: float
    itae: float
    rms_error: float
    overshoot_cm: float
    settling_time_s: float | None
    steady_state_error_cm: float
    control_travel_deg: float


def calculate_metrics(time, position, target, control, tolerance_cm: float = 0.5) -> PerformanceMetrics:
    t = np.asarray(time, dtype=float)
    y = np.asarray(position, dtype=float)
    r = np.asarray(target, dtype=float)
    u = np.asarray(control, dtype=float)
    if len(t) < 2:
        raise ValueError("At least two samples are required")

    dt = np.diff(t, prepend=t[0])
    dt[0] = dt[1]
    error = r - y
    abs_error = np.abs(error)
    iae = float(np.sum(abs_error * dt))
    ise = float(np.sum(error * error * dt))
    itae = float(np.sum(t * abs_error * dt))
    rms = float(np.sqrt(np.mean(error * error)))

    initial_direction = np.sign(r[0] - y[0])
    if initial_direction >= 0:
        overshoot = max(0.0, float(np.max(y - r)))
    else:
        overshoot = max(0.0, float(np.max(r - y)))

    outside = np.where(abs_error > tolerance_cm)[0]
    if len(outside) == 0:
        settling = 0.0
    elif outside[-1] >= len(t) - 1:
        settling = None
    else:
        settling = float(t[outside[-1] + 1])

    tail = max(1, len(error) // 10)
    ss_error = float(np.mean(np.abs(error[-tail:])))
    control_travel = float(np.sum(np.abs(np.diff(u)))) if len(u) > 1 else 0.0

    return PerformanceMetrics(iae, ise, itae, rms, overshoot, settling, ss_error, control_travel)
