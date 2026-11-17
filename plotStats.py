import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("stats.csv", skipinitialspace=True)

fig, (ax1, ax2) = plt.subplots(2)

ax1.plot(df["time"], df["velocity"])
ax1.set_title("Velocity")
ax1.set_xlabel("Time [s]")

ax2.plot(df["time"], df["acceleration"])
ax2.set_title("Acceleration")
ax2.set_xlabel("Time [s]")

plt.show()
