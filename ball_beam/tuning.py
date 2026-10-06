from dataclasses import replace
from typing import Iterable

from .config import PIDConfig, PlantConfig, SensorConfig
from .simulation import run_simulation


DEFAULT_SCENARIOS = (
    (10.0, 32.0, 0.0),
    (35.0, 12.0, 0.0),
    (5.0, 22.5, 8.0),
    (40.0, 20.0, -8.0),
)


def evaluate_gains(
    gains: tuple[float, float, float],
    scenarios: Iterable[tuple[float, float, float]] = DEFAULT_SCENARIOS,
    duration_s: float = 8.0,
    dt: float = 0.02,
    plant_config: PlantConfig | None = None,
    sensor_config: SensorConfig | None = None,
) -> float:
    kp, ki, kd = gains
    pid = PIDConfig(kp=max(0.0, kp), ki=max(0.0, ki), kd=max(0.0, kd))
    total = 0.0
    count = 0
    for scenario_id, (start, target, angle) in enumerate(scenarios):
        result = run_simulation(
            target_cm=target,
            initial_position_cm=start,
            initial_angle_deg=angle,
            duration_s=duration_s,
            dt=dt,
            seed=100 + scenario_id,
            plant_config=plant_config,
            sensor_config=sensor_config,
            pid_config=pid,
        )
        # ITAE dominates; add small penalties for actuator travel and final error.
        total += result.metrics.itae + 0.01 * result.metrics.control_travel_deg + 5.0 * result.metrics.steady_state_error_cm
        count += 1
    return total / max(1, count)


def twiddle(
    initial: tuple[float, float, float] = (4.1, 1.5, 1.46),
    deltas: tuple[float, float, float] = (0.5, 0.25, 0.25),
    tolerance: float = 0.05,
    max_iterations: int = 60,
):
    params = list(initial)
    dp = list(deltas)
    best = evaluate_gains(tuple(params))
    history = [{"iteration": 0, "kp": params[0], "ki": params[1], "kd": params[2], "cost": best}]

    iteration = 0
    while sum(dp) > tolerance and iteration < max_iterations:
        iteration += 1
        for i in range(3):
            params[i] += dp[i]
            params[i] = max(0.0, params[i])
            cost = evaluate_gains(tuple(params))
            if cost < best:
                best = cost
                dp[i] *= 1.1
            else:
                params[i] -= 2.0 * dp[i]
                params[i] = max(0.0, params[i])
                cost = evaluate_gains(tuple(params))
                if cost < best:
                    best = cost
                    dp[i] *= 1.1
                else:
                    params[i] += dp[i]
                    params[i] = max(0.0, params[i])
                    dp[i] *= 0.8
        history.append({"iteration": iteration, "kp": params[0], "ki": params[1], "kd": params[2], "cost": best})

    return tuple(params), history
