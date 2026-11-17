import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt

folder = Path("throttleData")

df = pd.concat(
    (
        pd.read_csv(f).assign(throttle=float(f.stem.removeprefix("throttle")))
        for f in sorted(folder.glob("throttle*.csv"))
    ),
    ignore_index=True,
)

fig, (ax1, ax2) = plt.subplots(2)

for throttle, g in df.groupby("throttle"):
    ax1.plot(g["Time"], g["Velocity"], label=throttle)
    ax2.plot(g["Velocity"], g["Acceleration"], label=throttle)

ax1.legend()
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Velocity [m/s]")

ax2.legend()
ax2.set_xlabel("Velocity [m/s]")
ax2.set_ylabel("Acceleration [m/s^2]")

plt.show()

