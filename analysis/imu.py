import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('imu.csv')

ax = df.plot(x='time', y='acc y')

plt.show()

ax = df.plot(x='time')

plt.show()
