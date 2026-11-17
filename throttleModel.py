import numpy as np
import pandas as pd
from pathlib import Path

folder = Path("throttleData")

df = pd.concat(
    (
        pd.read_csv(f).assign(throttle=float(f.stem.removeprefix("throttle")))
        for f in sorted(folder.glob("throttle*.csv"))
    ),
    ignore_index=True,
)

# skip start-up ringing, the speed limiter, and clamped throttle
d = df[(df["Velocity"] > 7) & (df["Velocity"] < 27) & (df["throttle"] < 0.95)]
u = d["throttle"].to_numpy()
v = d["Velocity"].to_numpy()
a = d["Acceleration"].to_numpy()

print("a = G*u - k*v^2")
A = np.column_stack([u, -(v**2)])
(G, k), *_ = np.linalg.lstsq(A, a, rcond=None)
rms = np.sqrt(np.mean((A @ [G, k] - a) ** 2))

print(f"G={G:.4f}  k={k:.6f}  RMS={rms:.4f}")
