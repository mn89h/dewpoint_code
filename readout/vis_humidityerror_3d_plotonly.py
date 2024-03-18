import math
import matplotlib.pyplot as plt
import numpy as np
import pickle

from matplotlib import cm
from matplotlib.ticker import LinearLocator
from mpl_toolkits.mplot3d import axes3d

# # try to plot 
# surf = ax.plot_surface(Ygrid, Xgrid, errors, cmap=cm.coolwarm,
#                        lw=0.5, rstride=200, cstride=200, edgecolor='royalblue',
#                        antialiased=False, alpha=0.5)

 
ax = plt.figure().add_subplot(projection='3d')
ax = pickle.load(open("plot.pickle", "rb"))

ax.set_xlabel("Humidity [\%]")
ax.set_ylabel("Temperature [°C]")
ax.set_zlabel("Relative RH Error [\%]", labelpad=0)
ax.view_init(20, 135, 0)
ax.autoscale()

ax.autoscale_view()
plt.tight_layout()
plt.show()
# plt.savefig('foo.png')
# plt.savefig('foo.pdf')
# plt.show()



# PS_ref = 100 * 6.0328 * np.exp((17.1485*X)/(234.69+X))
# print(np.min(PS_ref))
# print(np.max(PS_ref))