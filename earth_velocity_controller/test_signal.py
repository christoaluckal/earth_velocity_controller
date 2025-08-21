import numpy as np
import matplotlib.pyplot as plt

def generate_sine(min_range=0, max_range=1, duration=10, control_freq=10, wave_freq=1, phase=0, noise=0.02):

    dt = 1 / control_freq
    t = np.arange(0, duration, dt)
    y = np.sin(2 * np.pi * wave_freq * t + phase)
    amplitude = (max_range - min_range) / 2
    offset = (max_range + min_range) / 2
    y = y * amplitude + offset
    if noise > 0:
        y += np.random.uniform(-noise, noise, size=y.shape)
        y = np.clip(y, -1, 1)

    return t, y


if __name__ == "__main__":
    d = 60
    freq = 1/30
    t, y = generate_sine(min_range=-1, max_range=1, duration=d, control_freq=20, wave_freq=freq, phase=0, noise=0.0)
    plt.plot(t, y)
    plt.show()