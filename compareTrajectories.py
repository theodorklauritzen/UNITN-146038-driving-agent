from plotTrajectory import parse_trajectory, evalPolynom
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def main():
    trajectories = [
        "VehicleTrajectory(startTime=0.049, duration=7.5, startDistance=0, m=[-8.83774e-08, -1.76755e-06, 2.13333, -0.853333, 0.113778])",
        "VehicleTrajectory(startTime=9.249, duration=12.0931, startDistance=85.479, m=[15.0077, -0.0102472, -1.22636, 0.40606, -0.0503839])"
    ]

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

        label = f"Reference {i + 1}"
        ax1.plot(sTraj + startDistance, vTraj, "--", label=label)
        ax2.plot(time + startTime, vTraj, "--", label=label)

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

