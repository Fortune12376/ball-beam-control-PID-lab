from dataclasses import replace

from .config import PIDConfig


class PIDController:
    """PID with derivative-on-measurement and saturation-aware anti-windup."""

    def __init__(self, config: PIDConfig | None = None) -> None:
        self.cfg = config or PIDConfig()
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.last_output = 0.0

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self.cfg = replace(self.cfg, kp=kp, ki=ki, kd=kd)

    def compute(
        self,
        setpoint_cm: float,
        measured_position_cm: float,
        measured_velocity_cm_s: float,
        dt: float,
    ) -> float:
        if dt <= 0:
            raise ValueError("dt must be positive")

        error = setpoint_cm - measured_position_cm
        candidate_integral = self.integral + error * dt
        candidate_integral = min(
            max(candidate_integral, -self.cfg.integral_limit),
            self.cfg.integral_limit,
        )

        def raw(i_term: float) -> float:
            return (
                self.cfg.kp * error
                + self.cfg.ki * i_term
                - self.cfg.kd * measured_velocity_cm_s
            )

        unsaturated = raw(candidate_integral)
        limit = self.cfg.output_limit_deg
        saturated = min(max(unsaturated, -limit), limit)

        # Conditional integration: accept the new integral unless saturation
        # and the error are pushing further into the saturated direction.
        pushing_high = unsaturated > limit and error > 0
        pushing_low = unsaturated < -limit and error < 0
        if not (pushing_high or pushing_low):
            self.integral = candidate_integral
            unsaturated = raw(self.integral)
            saturated = min(max(unsaturated, -limit), limit)

        self.last_output = saturated
        return saturated
