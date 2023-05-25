import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import numpy as np


fig, ax = plt.subplots(subplot_kw={"projection": "3d"})


X = np.arange(0, 10, 0.01)
Y = np.arange(0, 100, 0.01)
X, Y = np.meshgrid(X, Y)
ymean = X ** 2
s = 10
Z = np.exp(-((Y-ymean)/s)**2/ 2) / (s * np.sqrt(2 * 3.14))

# Plot the surface.
surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

# Customize the z axis.
# ax.set_zlim(-1.01, 1.01)
# ax.zaxis.set_major_locator(LinearLocator(10))
# # A StrMethodFormatter is used automatically
# ax.zaxis.set_major_formatter('{x:.02f}')

# Add a color bar which maps values to colors.
# fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show(block = True)
