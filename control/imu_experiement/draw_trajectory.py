import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

def plot_trajectory(df, mode="3d", title="Trajectory"):
    # Ensure correct time format
    df["Time"] = pd.to_datetime(df["Time"], format="%H:%M:%S.%f")
    df["ElapsedTime"] = (df["Time"] - df["Time"].iloc[0]).dt.total_seconds()

    # Convert acceleration to m/s²
    accel = df[["AccelX", "AccelY", "AccelZ"]].to_numpy()
    time = df["ElapsedTime"].to_numpy()

    # Initialize velocity and position arrays
    velocity = np.zeros_like(accel)
    position = np.zeros_like(accel)

    # Compute velocity and position using trapezoidal integration
    for i in range(1, len(time)):
        dt = time[i] - time[i - 1]
        # velocity[i] = velocity[i - 1] + (accel[i] + accel[i - 1]) * dt
        # position[i] = position[i - 1] + (velocity[i] + velocity[i - 1]) * dt
        velocity[i] = velocity[i - 1] + accel[i - 1] * dt
        position[i] = position[i - 1] + velocity[i - 1] * dt

    # Plot based on mode
    if mode == "3d":
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(position[:, 0], position[:, 1], position[:, 2],
                label='Reconstructed Trajectory', linewidth=2)
        ax.scatter(position[:, 0], position[:, 1], position[:, 2],
                   color='black', s=10, label='Reconstructed Points')
        ax.set_title(title)
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_zlabel("Z Position (m)")
        # ax.legend()
    elif mode == "2d":
        plt.figure()
        plt.plot(position[:, 0], position[:, 1],
                 label='Reconstructed Trajectory', linewidth=2)
        plt.scatter(position[:, 0], position[:, 1],
                    color='black', s=10, label='Reconstructed Points')
        plt.title(title)
        plt.xlabel("X Position (cm)")
        plt.ylabel("Y Position (cm)")
        plt.axis('equal')
        plt.grid()
        # plt.legend()
    else:
        raise ValueError("Mode must be '2d' or '3d'.")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    df = pd.read_csv("circle_data.csv")
    plot_trajectory(df, mode="3d", title="Circular trajectory") 
    plot_trajectory(df, mode="2d", title="Circular trajectory") 

    df = pd.read_csv("square_data.csv")
    plot_trajectory(df, mode="3d", title="Square trajectory") 
    plot_trajectory(df, mode="2d", title="Square trajectory") 

    df = pd.read_csv("straightline_dataset.csv")
    plot_trajectory(df, mode="3d", title="Line trajectory") 
    plot_trajectory(df, mode="2d", title="Line trajectory") 

