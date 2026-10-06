from dataclasses import dataclass


@dataclass(frozen=True)
class PlantConfig:
    beam_length_cm: float = 45.0
    gravity_cm_s2: float = 981.0
    rolling_factor: float = 5.0 / 7.0
    friction_s_inv: float = 0.35
    servo_max_angle_deg: float = 45.0
    servo_speed_deg_s: float = 300.0
    restitution: float = 0.0
    stiction_angle_deg: float = 1.0
    stiction_velocity_cm_s: float = 0.1


@dataclass(frozen=True)
class SensorConfig:
    noise_sigma_cm: float = 0.4
    resolution_cm: float = 0.1
    samples_per_control_step: int = 4


@dataclass(frozen=True)
class PIDConfig:
    kp: float = 4.10
    ki: float = 1.50
    kd: float = 1.46
    integral_limit: float = 30.0
    output_limit_deg: float = 45.0


@dataclass(frozen=True)
class KalmanConfig:
    position_process_var: float = 0.02
    velocity_process_var: float = 0.20
    initial_position_var: float = 4.0
    initial_velocity_var: float = 25.0
