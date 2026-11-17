from plotTrajectory import parse_trajectory, evalPolynom
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def main():

    params = parse_trajectory("VehicleTrajectory(startTime=0.049, duration=7.5, m=[-8.83774e-08, -1.76755e-06, 2.13333, -0.853333, 0.113778])")
    duration = params["duration"]
    m = params["m"]
    startTime = params["startTime"]

    time = np.linspace(0, duration, 200)

    sTraj = evalPolynom(time, 0, m)
    vTraj = evalPolynom(time, 1, m)

    df = pd.read_csv("stats.csv", skipinitialspace=True)

    fig, (ax1, ax2) = plt.subplots(2)

    ax1.plot(sTraj, vTraj, label="Reference")
    ax1.plot(df["distance"], df["velocity"], label="Simulation")

    ax1.legend()
    ax1.set_xlabel("Distance [m]")
    ax1.set_ylabel("Velocity [m/s]")
    ax1.grid()

    ax2.plot(time + startTime, vTraj, label="Reference")
    ax2.plot(df["time"], df["velocity"], label="Simulation")

    ax2.legend()
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Velocity [m/s]")
    ax2.grid()

    plt.show()

if __name__ == "__main__":
    main()
