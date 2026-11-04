import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("stats.csv", skipinitialspace=True)

df.plot(x="time", y="velocity")
plt.xlabel("Time [s]")
plt.show()
