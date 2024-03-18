""" 
Visualize avg, min and max deviations from anticipated absolute/relative humidities over all measurements.
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FormatStrFormatter
import StyleHelper as SH
import scienceplots
import matplotlib.ticker as ticker
plt.style.use(['science','ieee'])
plt.rcParams.update({'figure.dpi': '100'})

min_dew = [-0.553105092,-0.993105092,-0.332707827]
min_rh = [-2.974401551,-5.166615369,-1.481737032]

max_dew = [1.181886236,0.164266832,2.901886236]
max_rh = [4.44480174,1.219431639,8.688061374]

avg_dew = [0.24838216,-0.32411784,0.330048827]
avg_rh = [0.877280499,-1.319768248,1.114157591]

std_dew = [0.508439555,0.345335048,0.812456951]
std_rh = [2.127760067,1.867696456,2.602150724]


x = np.array([-1,0,1])

paint = [SH.IES_YELLOW_100, SH.IES_BLUE_100]

sz = (8,4)

dew_err_mean = avg_dew
dew_err_bot = [avg_dew[i] - min_dew[i] for i in range(3)]
dew_err_top = [max_dew[i] - avg_dew[i] for i in range(3)]

rh_err_mean = avg_rh
rh_err_bot = [avg_rh[i] - min_rh[i] for i in range(3)]
rh_err_top = [max_rh[i] - avg_rh[i] for i in range(3)]

linestyles = ["solid", "dashed", "dotted", "dashdot", "solid"]
colors = ['k', 'r', 'b', 'g', 'k']
labels = ['IDE', 'VCNL4040', 'VCNL36825T']

l = []
fig, (ax, ax2) = plt.subplots(1, 2, layout='constrained')
fig.set_figheight(1.3)
for t in range(3):
    eb = ax.errorbar(x[t], y=dew_err_mean[t], yerr=([dew_err_bot[t]], [dew_err_top[t]]), capsize=2, color=colors[t], elinewidth=1, fmt=' ')
    eb[-1][0].set_linestyle(linestyles[t])
    ax.scatter(x[t], y=dew_err_mean[t], color=colors[t], s=20, marker='_')
    l.append(ax.plot(0,0,linestyle=linestyles[t],label=labels[t],color=colors[t]))

l = []
# fig2, ax2 = plt.subplots(1, figsize=sz)
for t in range(3):
    eb = ax2.errorbar(x[t], y=rh_err_mean[t], yerr=([rh_err_bot[t]], [rh_err_top[t]]), capsize=2, color=colors[t], elinewidth=1, fmt=' ')
    eb[-1][0].set_linestyle(linestyles[t])
    ax2.scatter(x[t], y=rh_err_mean[t], color=colors[t], s=20, marker='_')
    l.append(ax2.plot(0,0,linestyle=linestyles[t],label=labels[t],color=colors[t]))


dew_rh_string = ['T$_{\\text{Dew}}$', 'RH']
ax.set_ylim((-2,3))
ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
ax.yaxis.grid()
ax.axhline(0, color='black', linewidth=0.5)
ax.set_xticks(np.arange(-1.5, 2.0, 0.5))
# ax.legend()
ax.set_ylabel('$\Delta$' + dew_rh_string[0] + ' ($^{\circ}$C)')
ax.yaxis.set_label_position("left")
ax.tick_params(labelbottom=False, labeltop=False, labelleft=True, labelright=False,
                     bottom=False, top=False, left=True, right=True)
# ax.minorticks_off()
ax.xaxis.set_tick_params(which='minor', bottom=False, top=False)

ax2.set_ylim((-6,9))
ax2.yaxis.grid()
ax2.axhline(0, color='black', linewidth=0.5)
ax2.set_xticks(np.arange(-1.5, 2.0, 0.5))
# ax2.legend()
ax2.set_ylabel('$\Delta$' + dew_rh_string[1] + ' (\%)')
ax2.yaxis.set_label_position("right")
ax2.tick_params(labelbottom=False, labeltop=False, labelleft=False, labelright=True,
                     bottom=False, top=False, left=True, right=True)
# ax2.minorticks_off()
ax2.xaxis.set_tick_params(which='minor', bottom=False, top=False)

# plt.figlegend(l, labels)
# plt.figlegend(l, labels, loc = 'lower center', ncol=3)
handles, labels = ax2.get_legend_handles_labels()
fig.legend(handles, labels, loc='upper center', ncol=3, bbox_to_anchor=(0.5, 1.15))
# ax.legend(loc='upper right', bbox_to_anchor=(0, 1.15),
        #   ncol=3, columnspacing=0.5, handletextpad=0.1)

fig.savefig('meas_output/sensorall_dew.jpg', dpi=600)