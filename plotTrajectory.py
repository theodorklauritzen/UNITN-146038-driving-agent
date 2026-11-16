import ast
import sys
import numpy as np
import matplotlib.pyplot as plt
from itertools import chain

def parse_trajectory(s: str) -> dict:
    call = ast.parse(s.strip(), mode="eval").body
    if not isinstance(call, ast.Call):
        raise ValueError("Expected a call like VehicleTrajectory(...)")
    return {kw.arg: ast.literal_eval(kw.value) for kw in call.keywords}

def evalPolynom(time: float, derivates: int, m: list[float]) -> float:
    coeffs = [
        1.0 / 1.0,
        1.0 / 2.0,
        1.0 / 6.0,
        1.0 / 24.0,
        1.0 / 120.0
    ]

    # 0 = distance, 1 = velocity, 2 = acceleration, 3 = jerk
    result = 0.0
    # Distance has no constant offset
    if derivates > 0:
        result += m[derivates - 1]

    for i in range(5 - derivates):
        result += m[derivates + i] * time ** (i + 1) * coeffs[i]

    return result

def main():
    if len(sys.argv) != 2:
        sys.exit('Usage: python parse.py "VehicleTrajectory(startTime=0, duration=9.375, m=[...])"')

    params = parse_trajectory(sys.argv[1])
    duration = params["duration"]
    m = params["m"]

    print(f"duration={duration}")
    print(f"m={m}")

    time = np.linspace(0, duration, 200)

    fig, axes = plt.subplots(2, 2)

    titles = ["Distance", "Velocity", "Acceleration", "Jerk"]

    for i, ax in enumerate(chain.from_iterable(axes)):
        ax.plot(time, evalPolynom(time, i, m))
        ax.set_title(titles[i])

    plt.show()

if __name__ == "__main__":
    main()
