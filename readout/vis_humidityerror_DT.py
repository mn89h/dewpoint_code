import matplotlib as mpl
# Use the pgf backend (must be set before pyplot imported)
PLOTOPTION = 'pdf'
PLOTSIZE = 1
if PLOTOPTION == 'pgf':
    mpl.use('pgf')

import math
import matplotlib.pyplot as plt
import numpy as np
import pickle

from matplotlib import cm
from matplotlib.ticker import LinearLocator
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.axes_grid1 import make_axes_locatable

import StyleHelper as SH
SH.update_pyplot_rcParams()

cmap = mpl.colors.LinearSegmentedColormap.from_list("", [SH.IES_YELLOW_100, SH.IES_RED_100, SH.IES_BLUE_100])

# temperature setting [°C]
T_ERROR = 0.2
T_PLOT_MIN = -10
T_PLOT_MAX = 40
T_PLOT_STP = 0.2
T_CALC_MIN = -270
T_CALC_MAX = 80 + T_ERROR
T_CALC_STP = 0.005

# humidity setting [Rel.%]
H_MIN = 20
H_MAX = 100
H_STP = 0.05

# def find_nearest_index(array: np.array, value: int):
#     array = np.asarray(array)
#     idx = (np.abs(array - value)).argmin()
#     return idx

def find_nearest_index(array: np.array, value: int):
    idx = np.searchsorted(array, value, side="left")
    if idx > 0 and (idx == len(array) or math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
        return idx-1
    else:
        return idx

fig = plt.figure(figsize=SH.set_size(SH.TEXTWIDTH_MASTER * PLOTSIZE, 1, (1,1), 0.7), layout='constrained')
# ax = fig.add_axes([0, 0.2, 1, 0.8], projection='3d')
ax = fig.add_subplot(projection='3d')
ax.set_xlabel("Humidity [\\unit{\percent}]")
ax.set_ylabel("Temperature [\\unit{\celsius}]")
# ax.set_zlabel("Relative RH Error [%]")
# # Customize the z axis.
# ax.set_zlim(0, 2.4)

# # Plot the surface.
# surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
#                        linewidth=0, antialiased=False)

# ax.zaxis.set_major_locator(LinearLocator(10))
# # A StrMethodFormatter is used automatically
# ax.zaxis.set_major_formatter('{x:.02f}')

# # Add a color bar which maps values to colors.
# fig.colorbar(surf, shrink=0.5, aspect=5)

# plt.show()

# Get saturation water vapor pressure in hPa for all temperatures
# Sonntag 1990, Sources: 
# https://www.osti.gov/servlets/purl/548871
# https://sciencepolicy.colorado.edu/~voemel/vp.html
# https://www.dwd.de/DE/forschung/atmosphaerenbeob/lindenbergersaeule/rao_download/aktuell_2019_02.pdf?__blob=publicationFile&v=1
ENHANCEMENT_FACTOR = 1.005 # for air pressure = 1000 hPa
K1w = -6096.9385    # * T90^-1
K2w = 16.635794     #
K3w = -2.711193E-2  # * T90
K4w = 1.673952E-5   # * T90^2
K5w = 2.433502      # * ln(T90)
K1i = -6024.5282    # * T90^-1
K2i = 24.721994     #
K3i = 1.0613868E-2  # * T90
K4i = -1.3198825E-5 # * T90^2
K5i = -0.49382577   # * ln(T90)

# define temperature ranges to analyze (Xi over ice, Xw over water)
Xi = np.arange(T_PLOT_MIN, 0.01, T_PLOT_STP)
Xw = np.arange(0.01, T_PLOT_MAX+T_PLOT_STP, T_PLOT_STP)
# concatenate Xi and Xw for usage in plot axis
X = np.concatenate((Xi, Xw))

# process Xi and Xw for calculations in kelvin, use extended Xi for dew points of lowest Xi temperature values
Xi = Xi + 273.15
Xw = Xw + 273.15
Xiextd = np.arange(T_CALC_MIN, 0.01, T_CALC_STP)
Xiextd = Xiextd + 273.15
Xwextd = np.arange(0.01, T_CALC_MAX+T_CALC_STP, T_CALC_STP)
Xwextd = Xwextd + 273.15
Xextd = np.concatenate((Xiextd, Xwextd))

# actual saturation water vapor pressure calculation
PSiextd = ENHANCEMENT_FACTOR * np.exp(K1w/Xiextd + K2w + K3w*Xiextd + K4w*Xiextd*Xiextd + K5w*np.log(Xiextd))
PSwextd = ENHANCEMENT_FACTOR * np.exp(K1w/Xwextd + K2w + K3w*Xwextd + K4w*Xwextd*Xwextd + K5w*np.log(Xwextd))
PSi = ENHANCEMENT_FACTOR * np.exp(K1w/Xi + K2w + K3w*Xi + K4w*Xi*Xi + K5w*np.log(Xi))
PSw = ENHANCEMENT_FACTOR * np.exp(K1w/Xw + K2w + K3w*Xw + K4w*Xw*Xw + K5w*np.log(Xw))
PS = np.concatenate((PSiextd, PSwextd))

# Get saturation water vapor pressure for limited range (-40 to 80°C) and get the equilibrium vapor pressure at the respective dew point for all humidity levels
PS_ref = np.concatenate((PSi, PSw))
print(np.min(PS_ref))
print(np.max(PS_ref))
Y = np.arange(H_MIN, H_MAX, H_STP)
H = Y / 100
PS_dew_correct = np.outer(H, PS_ref)
PS_dew_1poff = np.outer(H+0.01, PS_ref)
errors = np.empty([len(H), len(PS_ref)])
print(len(PS))
for i in range(len(H)):
    for j in range(len(PS_ref)):
        dewpoint_temp_index = find_nearest_index(PS, PS_dew_correct[i][j])
        dewpoint_temp_error_index  = find_nearest_index(PS, PS_dew_1poff[i][j])
        # error = PS[dewpoint_temp_error_index] / PS_ref[j] - H[i]
        error = Xextd[dewpoint_temp_error_index] - Xextd[dewpoint_temp_index]
        # if j == 1000:
        #     print(PS[dewpoint_temp_error_index])
        #     print(PS_ref[j])
        #     print(error)
        errors[i][j] = np.abs(error)
    print(i/len(H) * 100)
# errors = errors * 100
# print(PS_dew.shape)

Xgrid, Ygrid = np.meshgrid(X, Y)

# try to plot 
# surf = ax.plot_surface(Ygrid, Xgrid, errors, cmap=cm.coolwarm,
#                        lw=0.5, rstride=200, cstride=200, edgecolor='royalblue',
#                        antialiased=False, alpha=0.5)
surf = ax.plot_surface(Ygrid, Xgrid, errors, cmap=cmap,
                        # lw=0.1, rstride=160, cstride=50, edgecolor='royalblue',
                       antialiased=True)
# ax.grid(axis='y')

pickle.dump(ax, open("plot.pickle", "wb"))

ax.view_init(20, 45, 0)

# ax.invert_xaxis()
# ax.invert_yaxis()
# ax.set_box_aspect(aspect=None, zoom=0.8)
# ax.relim()
# ax.autoscale_view()
# plt.tight_layout()
fig.colorbar(plt.cm.ScalarMappable(cmap = cmap), ax = [ax], fraction=0.018, pad=0.05, ticks=None, location='right', label='Max. Dew Point Error [\\unit{\celsius}]')

# fig.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

if PLOTOPTION == 'pgf':
    plt.savefig('lolza.pgf', format = 'pgf')
else:
    plt.savefig('dew_deviation.pdf', bbox_inches='tight')
    plt.show()

# PS_ref = 100 * 6.0328 * np.exp((17.1485*X)/(234.69+X))
# print(np.min(PS_ref))
# print(np.max(PS_ref))