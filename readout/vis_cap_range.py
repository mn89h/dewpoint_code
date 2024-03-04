# -*- coding: utf-8 -*-
"""
Created on Sun Mar  3 18:41:43 2024

@author: malte
"""

import matplotlib as mpl
# Use the pgf backend (must be set before pyplot imported)
PLOTOPTION = 'pdf'
PLOTSIZE = 1
if PLOTOPTION == 'pgf':
    mpl.use('pgf')
    
import StyleHelper as SH
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

SH.update_pyplot_rcParams()


# Path to the folder containing the CSV files
folder_path = 'C:/Users/malte/Documents/master/measurements/thesis'

# Initialize an empty array to store the last column values
names = []
capacitances = []
luminances = []

# Loop through each file in the folder
for file_name in os.listdir(folder_path):
    if file_name.endswith('.csv'):
        file_path = os.path.join(folder_path, file_name)
        names.append(file_name)
        
        with open(file_path, 'r') as file:
            csv_reader = csv.reader(file)
            
            # Read the first five rows of the CSV file
            c_values = []
            l_values = []
            for _ in range(5):
                row = next(csv_reader, None)
                if row is None:
                    break
                
                # Get the last column value
                c_values.append(float(row[-1]))
                l_values.append(float(row[-2]))
            capacitances.append(c_values)
            luminances.append(l_values)

capacitances_mean = np.empty(len(capacitances))
for i, arr in enumerate(capacitances):
    capacitances_mean[i] = np.mean(arr)
    
luminances_mean = np.empty(len(luminances))
for i, arr in enumerate(luminances):
    luminances_mean[i] = np.mean(arr)
    

c_matrix = capacitances_mean.reshape(3, 4)
l_matrix = luminances_mean.reshape(3, 4)

  
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

def vapor_pressure_water(T):
    return ENHANCEMENT_FACTOR * np.exp(K1w/T + K2w + K3w*T + K4w*T*T + K5w*np.log(T))

def get_p(RH, T_C):
    T = T_C + 273.15
    p = RH * vapor_pressure_water(T)
    return p

def get_AH(RH, T_C):
    p = get_p(RH, T_C)
    T = T_C + 273.15
    AH = p * 100 / (461.5 * T) * 1000 # g/m^3
    return AH

# Data for the plot
x = [10, 25, 40]  # Temperature values
y1 = [[33.47, 57.36, 75.67, 98.18], 
      [32.78, 52.89, 75.29, 97.59], 
      [31.60, 48.42, 74.68, 96.71]]  # Humidity values
y2 = [[get_AH(rh/100, t) for rh in y1[i_t]] for i_t, t in enumerate(x)]
print(y2)
z1 = c_matrix  # Provided values
z2 = l_matrix

# Create a figure and a 3D axis
fig, ax = plt.subplots(2,2, figsize=SH.set_size(SH.TEXTWIDTH_MASTER * PLOTSIZE))
colors = [SH.IES_BLUE_100, SH.IES_YELLOW_100, SH.IES_RED_100]
markers = [['s', 's', 'o', 'o'], ['s', 'o', 'o', 'o'], ['s', 'o', 'o', 'o']]

# Plot for RH
for i, xi in enumerate(x):
    ax[0,0].plot(y1[i], z1[i], c=colors[i], zorder=1)
    ax[1,0].plot(y1[i], z2[i], c=colors[i], zorder=1)
    SH.mscatter(y1[i], z1[i], c=colors[i], m=markers[i], ax=ax[0,0], zorder=2)
    SH.mscatter(y1[i], z2[i], c=colors[i], m=markers[i], ax=ax[1,0], zorder=2)
    # ax[0,0].scatter(y1[i], z1[i], c=colors[i], marker=markers[i])
    # ax[1,0].scatter(y1[i], z2[i], c=colors[i])

# Plot for AH
for i, xi in enumerate(x):
    ax[0,1].plot(y2[i], z1[i], c=colors[i], zorder=1)
    ax[1,1].plot(y2[i], z2[i], c=colors[i], zorder=1, label='\\qty{' + str(xi) +'}{\celsius}')
    SH.mscatter(y2[i], z1[i], c=colors[i], m=markers[i], ax=ax[0,1], zorder=1)
    SH.mscatter(y2[i], z2[i], c=colors[i], m=markers[i], ax=ax[1,1], zorder=1)
    # ax[0,1].scatter(y2[i], z1[i], c=colors[i])
    # ax[1,1].scatter(y2[i], z2[i], label=str(xi) + '°C', c=colors[i])

# Set labels for each axis
ax[0,0].set_ylabel('Frequency [\\unit{\hertz}]')
ax[1,0].set_ylabel('Light Value')
ax[1,0].set_xlabel('Relative Humidity [\\unit{\percent}]')
ax[1,1].set_xlabel('Absolute Humidity [\\unit{\g \per \m^3}]')

ax[0,1].yaxis.set_ticklabels([])
ax[1,1].yaxis.set_ticklabels([])

ax[1,1].legend()

ax[0,0].margins(0.05, 0.1)
ax[0,1].margins(0.05, 0.1)
ax[1,0].margins(0.05, 0.1)
ax[1,1].margins(0.05, 0.1)
fig.tight_layout()


# # recompute the ax.dataLim
# [[ax[i,j].relim() for j in range(len(ax[i]))] for i in range(len(ax))]
# [[ax[i,j].autoscale_view() for j in range(len(ax[i]))] for i in range(len(ax))]

if PLOTOPTION == 'pgf':
    plt.savefig('lolza.pgf', format = 'pgf')
else:
    plt.savefig('start_range_cap_light.pdf', bbox_inches='tight')
    plt.show()