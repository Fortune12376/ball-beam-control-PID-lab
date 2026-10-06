from ball_beam.tuning import twiddle


if __name__ == "__main__":
    gains, history = twiddle()
    print(f"Best gains: Kp={gains[0]:.4f}, Ki={gains[1]:.4f}, Kd={gains[2]:.4f}")
    print(f"Final cost: {history[-1]['cost']:.3f}")
