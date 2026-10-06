from dataclasses import dataclass
import numpy as np

from .config import KalmanConfig, PIDConfig, PlantConfig, SensorConfig
from .controller import PIDController
from .filters import BallBeamKalmanFilter
from .metrics import PerformanceMetrics, calculate_metrics
from .physics import BallBeamPlant


@dataclass
class SimulationResult:
    time: np.ndarray
    true_position: np.ndarray
    true_velocity: np.ndarray
    measured_position: np.ndarray
    estimated_position: np.ndarray
    estimated_velocity: np.ndarray
    beam_angle: np.ndarray
    commanded_angle: np.ndarray
    target: np.ndarray
    metrics: PerformanceMetrics


def run_simulation(
    *,
    target_cm: float = 30.0,
    initial_position_cm: float = 8.0,
    initial_angle_deg: float = 0.0,
    duration_s: float = 10.0,
    dt: float = 0.01,
    seed: int = 42,
    plant_config: PlantConfig | None = None,
    sensor_config: SensorConfig | None = None,
    pid_config: PIDConfig | None = None,
    kalman_config: KalmanConfig | None = None,
) -> SimulationResult:
    plant_cfg = plant_config or PlantConfig()
    sensor_cfg = sensor_config or SensorConfig()
    pid_cfg = pid_config or PIDConfig()

    if not 0 <= target_cm <= plant_cfg.beam_length_cm:
        raise ValueError("target_cm must be on the beam")
    if not 0 <= initial_position_cm <= plant_cfg.beam_length_cm:
        raise ValueError("initial_position_cm must be on the beam")

    plant = BallBeamPlant(plant_cfg, sensor_cfg, initial_position_cm, seed)
    plant.reset(initial_position_cm, initial_angle_deg)
    controller = PIDController(pid_cfg)
    estimator = BallBeamKalmanFilter(initial_position_cm, plant_cfg, sensor_cfg, kalman_config)

    steps = int(round(duration_s / dt))
    logs = {name: [] for name in (
        "time", "true_position", "true_velocity", "measured_position",
        "estimated_position", "estimated_velocity", "beam_angle",
        "commanded_angle", "target",
    )}

    for k in range(steps + 1):
        t = k * dt
        measurement = plant.read_position_average()
        est_pos, est_vel = estimator.update(measurement, plant.angle_deg, dt)
        command = controller.compute(target_cm, est_pos, est_vel, dt)
        plant.command_angle(command)

        logs["time"].append(t)
        logs["true_position"].append(plant.position_cm)
        logs["true_velocity"].append(plant.velocity_cm_s)
        logs["measured_position"].append(measurement)
        logs["estimated_position"].append(est_pos)
        logs["estimated_velocity"].append(est_vel)
        logs["beam_angle"].append(plant.angle_deg)
        logs["commanded_angle"].append(command)
        logs["target"].append(target_cm)

        if k < steps:
            plant.step(dt)

    arrays = {key: np.asarray(value, dtype=float) for key, value in logs.items()}
    metrics = calculate_metrics(
        arrays["time"], arrays["true_position"], arrays["target"], arrays["commanded_angle"]
    )
    return SimulationResult(**arrays, metrics=metrics)
