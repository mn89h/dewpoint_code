# -*- coding: utf-8 -*-
"""
Created on Wed Feb 28 21:52:21 2024

@author: malte
"""

import matplotlib as mpl
# Use the pgf backend (must be set before pyplot imported)
PLOTOPTION = 'pdf'
PLOTSIZE = 1
if PLOTOPTION == 'pgf':
    mpl.use('pgf')

import matplotlib.pyplot as plt
import numpy as np
import StyleHelper as SH
SH.update_pyplot_rcParams()



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


# def vapor_pressure(T):
#     return 6.112*np.exp(17.67*T/(T+243.5))


def vapor_pressure_ice(T):
    return ENHANCEMENT_FACTOR * np.exp(K1w/T + K2w + K3w*T + K4w*T*T + K5w*np.log(T))
    
def vapor_pressure_water(T):
    return ENHANCEMENT_FACTOR * np.exp(K1w/T + K2w + K3w*T + K4w*T*T + K5w*np.log(T))

Ti = np.arange(-10, 0.1, 0.1)
Tw = np.arange(0.1, 50.1, 0.1)
Ti = Ti + 273.15
Tw = Tw + 273.15
T = np.concatenate([Ti, Tw])
T_degC = T - 273.15
ei = vapor_pressure_ice(Ti)
ew = vapor_pressure_water(Tw)
e = np.concatenate([ei, ew])
rh = np.arange(1.0, 0, -0.1)
p = np.outer(rh, e)

# AH = m/V = p/(Rw*T) [g/m^3]
# AH = rh * e[-1] * 100 * 1000 / (461.5 * T[-1]) # 100 for hPa->Pa, 1000 for kg->g, 461.5 for specific gas constant
# print(e[-1])
# print(T[-1])
# print(AH)


AH = np.arange(10,110,10) # AH in g/m^3
AH_start = 10
AH_step = 10
p_AH = []
idx = []

AH = AH_start
AH_exists_in_graph = True
while AH_exists_in_graph:
    p_AHi = ((AH / 1000) * (461.5 * T) / 100)
    index = np.argwhere(np.diff(np.sign(p_AHi - e))).flatten()
    if len(index) > 0:
        idx.append(index)
        p_AH.append(p_AHi)
        AH += 10
    else:
        AH_exists_in_graph = False
# for idxi in idx:
#     print(T[idxi[0]] - 273.15)


fig, ax1 = plt.subplots(figsize=SH.set_size(SH.TEXTWIDTH_MASTER * PLOTSIZE))
ax1.set_xlabel('Temperature [\\unit{\celsius}]')
ax1.set_ylabel('Water Vapor Pressure [\\unit{\hecto\pascal}]')


# T_index = 12 + 273.15
# T_index = np.where(T == T_index)[0]
# print(T_index[0])
# print(p_AH[0][T_index][0])
# l2 = np.array((12 - 3, p_AH[0][T_index][0] + .5))
# print(l2)


# angle = 3
# th2 = ax1.text(*l2, '10', 
#               rotation=angle, rotation_mode='anchor',
#               transform_rotates_text=True)

for i, p_i in enumerate(p):
    if i == 0:
        ax1.plot(T_degC, p_i, color=SH.color(SH.IES_BLUE, 100), linestyle='solid', label='Saturation Vapor Pressure')
    elif i == 1:
        ax1.plot(T_degC, p_i, SH.color(SH.IES_BLUE, 30), linestyle='solid', label='Relative Humidity')
    else:
        ax1.plot(T_degC, p_i, SH.color(SH.IES_BLUE, 30), linestyle='solid')
    if i > 0:
        x_pos = T_degC[-1] + 0.5/PLOTSIZE
        y_pos = p_i[-1] -2/PLOTSIZE
        pos = np.array((x_pos, y_pos))
        angle = 0
        ax1.text(*pos, '\qty{' + str(-i * 10 + 100) + '}{\%}', color=SH.color(SH.IES_BLUE, 30), rotation=angle, rotation_mode='default', transform_rotates_text=True)
         

    
for i, p_AHi in enumerate(p_AH):
    if i == 0:
        ax1.plot(T_degC[idx[i][0]+1:], p_AHi[idx[i][0]+1:], SH.color(SH.IES_RED, 100), linestyle='dashed', label='Absolute Humidity [\\unit{g \per m^3}]')
    else:
        ax1.plot(T_degC[idx[i][0]+1:], p_AHi[idx[i][0]+1:], SH.color(SH.IES_RED, 100), linestyle='dashed')
    if (i < len(p_AH) - 1):
        x_pos = T_degC[idx[i][0]+1] - 2.5/PLOTSIZE
        y_pos = p_AHi[idx[i][0]+1] +1/PLOTSIZE
        pos = np.array((x_pos, y_pos))
        angle = 0
        ax1.text(*pos, '\\num{' + str(i * AH_step + AH_start) +'}', color=SH.color(SH.IES_RED, 100))
         
fig.tight_layout()

ax1.margins(0, 0)
# recompute the ax.dataLim
ax1.relim()
# update ax.viewLim using the new dataLim
ax1.autoscale_view()

plt.grid()
plt.legend()

if PLOTOPTION == 'pgf':
    plt.savefig('lolza.pgf', format = 'pgf')
else:
    plt.savefig('saturation_pressure_chart.pdf', bbox_inches='tight')
    plt.show()