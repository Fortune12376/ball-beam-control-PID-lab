from ball_beam.simulation import run_simulation


def main() -> None:
    result = run_simulation()
    m = result.metrics
    settling = "not settled" if m.settling_time_s is None else f"{m.settling_time_s:.2f} s"
    print("Ball-and-Beam PID demo")
    print(f"ITAE: {m.itae:.2f}")
    print(f"RMS error: {m.rms_error:.2f} cm")
    print(f"Overshoot: {m.overshoot_cm:.2f} cm")
    print(f"Settling time: {settling}")
    print(f"Steady-state error: {m.steady_state_error_cm:.2f} cm")


if __name__ == "__main__":
    main()
