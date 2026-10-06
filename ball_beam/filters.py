import math
import numpy as np

from .config import KalmanConfig, PlantConfig, SensorConfig


class BallBeamKalmanFilter:
    """Two-state EKF-like estimator for position and velocity.

    The state transition uses the nonlinear ball acceleration model while the
    covariance propagation uses a local constant-velocity Jacobian.
    """

    def __init__(
        self,
        initial_position_cm: float,
        plant_config: PlantConfig,
        sensor_config: SensorConfig,
        kalman_config: KalmanConfig | None = None,
    ) -> None:
        self.plant = plant_config
        self.sensor = sensor_config
        self.cfg = kalman_config or KalmanConfig()
        self.x = np.array([initial_position_cm, 0.0], dtype=float)
        self.P = np.diag([
            self.cfg.initial_position_var,
            self.cfg.initial_velocity_var,
        ]).astype(float)

    def update(self, measurement_cm: float, angle_deg: float, dt: float) -> tuple[float, float]:
        if dt <= 0:
            raise ValueError("dt must be positive")

        position, velocity = self.x
        acceleration = (
            self.plant.rolling_factor
            * self.plant.gravity_cm_s2
            * math.sin(math.radians(angle_deg))
            - self.plant.friction_s_inv * velocity
        )

        x_pred = np.array([
            position + velocity * dt + 0.5 * acceleration * dt * dt,
            velocity + acceleration * dt,
        ])

        # Linearized state Jacobian including viscous friction.
        f22 = 1.0 - self.plant.friction_s_inv * dt
        F = np.array([[1.0, dt], [0.0, f22]])
        Q = np.diag([
            self.cfg.position_process_var,
            self.cfg.velocity_process_var,
        ])
        p_pred = F @ self.P @ F.T + Q

        samples = max(1, self.sensor.samples_per_control_step)
        measurement_var = (self.sensor.noise_sigma_cm ** 2) / samples
        # Quantisation noise variance for a uniform quantisation error.
        measurement_var += (self.sensor.resolution_cm ** 2) / 12.0

        H = np.array([[1.0, 0.0]])
        innovation = float(measurement_cm - (H @ x_pred)[0])
        S = float((H @ p_pred @ H.T)[0, 0] + measurement_var)
        K = (p_pred @ H.T)[:, 0] / S

        self.x = x_pred + K * innovation
        I = np.eye(2)
        KH = np.outer(K, H[0])
        # Joseph form is numerically safer than (I-KH)P alone.
        self.P = (I - KH) @ p_pred @ (I - KH).T + np.outer(K, K) * measurement_var

        self.x[0] = min(max(self.x[0], 0.0), self.plant.beam_length_cm)
        return float(self.x[0]), float(self.x[1])
