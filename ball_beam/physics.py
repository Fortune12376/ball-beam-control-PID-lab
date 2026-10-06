import math
import random
from dataclasses import dataclass

from .config import PlantConfig, SensorConfig


@dataclass
class PlantState:
    position_cm: float
    velocity_cm_s: float
    angle_deg: float
    commanded_angle_deg: float


class BallBeamPlant:
    """Nonlinear ball-and-beam plant with servo dynamics and noisy sensing."""

    def __init__(
        self,
        plant_config: PlantConfig | None = None,
        sensor_config: SensorConfig | None = None,
        initial_position_cm: float = 20.0,
        seed: int | None = None,
    ) -> None:
        self.cfg = plant_config or PlantConfig()
        self.sensor_cfg = sensor_config or SensorConfig()
        self.rng = random.Random(seed)
        self.reset(initial_position_cm)

    @property
    def state(self) -> PlantState:
        return PlantState(
            self.position_cm,
            self.velocity_cm_s,
            self.angle_deg,
            self.commanded_angle_deg,
        )

    def reset(self, position_cm: float = 20.0, angle_deg: float = 0.0) -> None:
        self.position_cm = min(max(position_cm, 0.0), self.cfg.beam_length_cm)
        self.velocity_cm_s = 0.0
        self.angle_deg = min(max(angle_deg, -self.cfg.servo_max_angle_deg), self.cfg.servo_max_angle_deg)
        self.commanded_angle_deg = self.angle_deg

    def command_angle(self, angle_deg: float) -> float:
        limit = self.cfg.servo_max_angle_deg
        self.commanded_angle_deg = min(max(angle_deg, -limit), limit)
        return self.commanded_angle_deg

    def _acceleration(self, angle_deg: float, velocity_cm_s: float) -> float:
        gravity_component = (
            self.cfg.rolling_factor
            * self.cfg.gravity_cm_s2
            * math.sin(math.radians(angle_deg))
        )
        return gravity_component - self.cfg.friction_s_inv * velocity_cm_s

    def step(self, dt: float) -> PlantState:
        """Advance the nonlinear plant by one control interval using midpoint integration."""
        if dt <= 0:
            raise ValueError("dt must be positive")

        error_deg = self.commanded_angle_deg - self.angle_deg
        max_delta = self.cfg.servo_speed_deg_s * dt
        servo_delta = min(max(error_deg, -max_delta), max_delta)
        mid_angle = self.angle_deg + 0.5 * servo_delta

        a0 = self._acceleration(self.angle_deg, self.velocity_cm_s)
        mid_velocity = self.velocity_cm_s + 0.5 * a0 * dt
        amid = self._acceleration(mid_angle, mid_velocity)

        self.angle_deg += servo_delta
        self.velocity_cm_s += amid * dt
        self.position_cm += mid_velocity * dt

        if (
            abs(self.angle_deg) < self.cfg.stiction_angle_deg
            and abs(self.velocity_cm_s) < self.cfg.stiction_velocity_cm_s
        ):
            self.velocity_cm_s = 0.0

        if self.position_cm < 0.0:
            self.position_cm = 0.0
            self.velocity_cm_s = -self.velocity_cm_s * self.cfg.restitution
        elif self.position_cm > self.cfg.beam_length_cm:
            self.position_cm = self.cfg.beam_length_cm
            self.velocity_cm_s = -self.velocity_cm_s * self.cfg.restitution

        return self.state

    def read_position(self) -> float:
        noisy = self.position_cm + self.rng.gauss(0.0, self.sensor_cfg.noise_sigma_cm)
        resolution = self.sensor_cfg.resolution_cm
        if resolution > 0:
            noisy = round(noisy / resolution) * resolution
        return min(max(noisy, 0.0), self.cfg.beam_length_cm)

    def read_position_average(self, samples: int | None = None) -> float:
        n = samples or self.sensor_cfg.samples_per_control_step
        if n < 1:
            raise ValueError("samples must be >= 1")
        return sum(self.read_position() for _ in range(n)) / n
