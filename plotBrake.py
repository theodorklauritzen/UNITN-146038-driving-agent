import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt

folder = Path("brakeData")

df = pd.concat(
    (
        pd.read_csv(f).assign(brake=float(f.stem.removeprefix("brake")))
        for f in sorted(folder.glob("brake*.csv"))
    ),
    ignore_index=True,
)

df = df[df["time"] >= 4]

fig, (ax1, ax2) = plt.subplots(2)

for brake, g in df.groupby("brake"):
    ax1.plot(g["time"], g["velocity"], label=brake)
    ax2.plot(g["velocity"], g["acceleration"], label=brake)

ax1.legend()
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Velocity [m/s]")

ax2.legend()
ax2.set_xlabel("Velocity [m/s]")
ax2.set_ylabel("Acceleration [m/s^2]")

plt.show()

