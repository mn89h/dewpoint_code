# -*- coding: utf-8 -*-
"""
Created on Wed Feb 28 21:52:21 2024

@author: malte
"""

import matplotlib as mpl
# Use the pgf backend (must be set before pyplot imported)
PLOTOPTION = 'pdf'
PLOTSIZE = 0.5
if PLOTOPTION == 'pgf':
    mpl.use('pgf')

import csv
import matplotlib.pyplot as plt
import StyleHelper as SH
import pandas as pd
SH.update_pyplot_rcParams()

x = []
y = []

# Open the CSV file and read the data
with open('waterabsorptiondata.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        x.append(int(row[0]))  # Assuming the first column contains the x-axis data
        y.append(float(row[1]))  # Assuming the second column contains the y-axis data and converting it to float


# Create the plot
fig, ax1 = plt.subplots(figsize=SH.set_size(SH.TEXTWIDTH_MASTER * PLOTSIZE))
# plt.figure(figsize=(10, 6))
plt.yscale("log",subs=[1,2,3,4,5,6,7,8,9])
ax1.plot(x, y, linestyle='-', color=SH.IES_BLUE_100)  # 'o' marker and blue color

plt.axvline(x=380, linestyle='dotted')
plt.axvline(x=780, linestyle='dotted')
plt.text(180,1.5E2, "UV")
plt.text(815,1.5E2, "IR")
# ax1.set_yscale('log', subs=[1,2,3,4,5,6,7,8,9])  # Set the y-axis to a logarithmic scale
ax1.set_yticks((1E-1,1E1,1E3))
# plt.loglog(y)
# ax1.set_yticklabels(("1","10","100","1000"))
# plt.minorticks_on()
ax1.grid(True, which='both', axis='y')

# Add labels and title for clarity
ax1.set_xlabel('Wavelength \\begin{math}\lambda\end{math} [\\unit{\\nano\m}]')
ax1.set_ylabel('Absorption a\\textsubscript{w}($\mathrm{\lambda}$) [\\unit{\meter^{-1}}]')
# plt.title('Plot Title')  # Replace 'Plot Title' with your actual plot title




    
# for i, p_AHi in enumerate(p_AH):
#     if i == 0:
#         ax1.plot(T_degC[idx[i][0]+1:], p_AHi[idx[i][0]+1:], SH.color(SH.IES_RED, 100), linestyle='dashed', label='Absolute Humidity [\\unit{g \per m^3}]')
#     else:
#         ax1.plot(T_degC[idx[i][0]+1:], p_AHi[idx[i][0]+1:], SH.color(SH.IES_RED, 100), linestyle='dashed')
#     if (i < len(p_AH) - 1):
#         x_pos = T_degC[idx[i][0]+1] - 2.5/PLOTSIZE
#         y_pos = p_AHi[idx[i][0]+1] +1/PLOTSIZE
#         pos = np.array((x_pos, y_pos))
#         angle = 0
#         ax1.text(*pos, '\\num{' + str(i * AH_step + AH_start) +'}', color=SH.color(SH.IES_RED, 100))
         
# fig.tight_layout()

ax1.margins(0.02, 0.05)
# recompute the ax.dataLim
ax1.relim()
# update ax.viewLim using the new dataLim
ax1.autoscale_view()

# plt.grid()
# plt.legend()

if PLOTOPTION == 'pgf':
    plt.savefig('lolza.pgf', format = 'pgf')
else:
    plt.savefig('waterabsorption.pdf', bbox_inches='tight')
    plt.show()