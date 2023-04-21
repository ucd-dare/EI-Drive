import matplotlib.pyplot as plt
import pandas as pd


df = pd.read_csv('log_1.csv', index_col=False)

plt.plot(df.x, df.y)
ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
plt.show()