from plotTrajectory import parse_trajectory, evalPolynom
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def main():
    trajectories = []
    with open("trajectories.txt") as file:
        trajectories = file.read().strip().split("\n")

    df = pd.read_csv("stats.csv", skipinitialspace=True)

    fig, (ax1, ax2) = plt.subplots(2)

    for i, traj in enumerate(trajectories):
        params = parse_trajectory(traj)
        duration = params["duration"]
        m = params["m"]
        startTime = params["startTime"]
        startDistance = params["startDistance"]

        time = np.linspace(0, duration, 200)

        sTraj = evalPolynom(time, 0, m)
        vTraj = evalPolynom(time, 1, m)

        label = "Reference trajectory" if i == 0 else None
        ax1.plot(sTraj + startDistance, vTraj, "--", color="orange", label=label)
        ax2.plot(time + startTime, vTraj, "--", color="orange", label=label)

    ax1.plot(df["distance"], df["velocity"], label="Simulation")
    ax2.plot(df["time"], df["velocity"], label="Simulation")

    ax1.legend()
    ax1.set_xlabel("Distance [m]")
    ax1.set_ylabel("Velocity [m/s]")
    ax1.grid()

    ax2.legend()
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Velocity [m/s]")
    ax2.grid()

    plt.show()

if __name__ == "__main__":
    main()

