import numpy as np
import pandas as pd


def generate_ground_truth(nSamples):
    y = np.linspace(-400, 0, 18)
    x = np.ones(18, dtype=float) * 300.0

    turn_radius = 300.0  # m
    angular_range = np.pi * 0.5
    dtheta = angular_range / 17
    theta = np.linspace(dtheta, angular_range, 17)

    x_turn = turn_radius * np.cos(theta)
    y_turn = turn_radius * np.sin(theta)

    positions = np.zeros((2, nSamples))
    positions[0] = np.concatenate((x, x_turn))
    positions[1] = np.concatenate((y, y_turn))

    speed = np.zeros_like(positions)
    speed[0, :-1] = positions[0, 1:] - positions[0, :-1]
    speed[1, :-1] = positions[1, 1:] - positions[1, :-1]
    speed[:, -1] = speed[:, -2]

    acceleration = np.zeros_like(positions)
    acceleration[0, :-1] = speed[0, 1:] - speed[0, :-1]
    acceleration[1, :-1] = speed[1, 1:] - speed[1, :-1]

    gt_dict = {
        "px": positions[0],
        "py": positions[1],
        "vx": speed[0],
        "vy": speed[1],
        "ax": acceleration[0],
        "ay": acceleration[1],
    }
    df = pd.DataFrame(gt_dict)
    df.to_csv("data/simple_data/ground_truth.csv")


if __name__ == "__main__":
    generate_ground_truth()
