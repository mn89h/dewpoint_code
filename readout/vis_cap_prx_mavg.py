import csv
import os
import matplotlib.pyplot as plt
import datetime
import re
import numpy
from scipy.interpolate import splrep, BSpline, splev
from scipy.interpolate import CubicSpline
from scipy.signal import argrelextrema
import sys
import statistics as stat
from numpy import diff, gradient
import matplotlib

path = 'C:/Users/malte/Documents/master/measurements/thesis/'
# filename_in = 'hum_test38'
filename_in = 't10rh33.4'
fileext_in = '.txt'
filename_out = filename_in
# filename = 'pid_t0.2_kp20_ki5_kd30'

in_file = path + filename_in

disable_all_file_preprocessing = True
split_header = True
check_whitespace = True
col_as_offset = True

if (os.path.isfile(os.path.join(in_file + '.csv'))):
    print("File exists")
    disable_all_file_preprocessing = True
    fileext_in = '.csv'
    
in_full_path = os.path.join(path, filename_in + fileext_in)

class Constants:
    PERIOD          = 0.2
    COL_HUM         = 7 # in old files 5 due to missing T[0]
    COL_TEMP_HUM    = 2
    COL_CAP         = [-1]
    COL_TEMP_CAP    = [2]
    COL_PRX         = [-2,-3]
    COL_TEMP_PRX    = [1,1]
    DCAP_YLIM       = [[-100, 30]]
    DPRX_YLIM       = [[-50, 50], [-300, 300]]
    

out_full_path_csv = os.path.join(path, filename_out + '.csv')
out_full_path_txt = os.path.join(path, filename_out + '.txt')
out_full_path_png = os.path.join(path, filename_out + '.png')

if not disable_all_file_preprocessing:
    # Split the input file into csv (samples) and txt (info header)
    if (split_header):
        with open(in_full_path, 'r') as infile:
            lines = infile.readlines()
            
        with open(out_full_path_txt, 'w') as outfile1: 
            outfile1.writelines(lines[:4])
            
        with open(out_full_path_csv, 'w') as outfile2:
           outfile2.writelines(lines[4:-1])
    
    # Replace whitespace with commas
    if (check_whitespace):
        with open(out_full_path_csv, 'r') as f:
            filedata = f.read()
        
        newdata = re.sub(r' +', ',', filedata)
        
        with open(out_full_path_csv, 'w') as f:
            f.write(newdata)

# Count lines & columns
if (col_as_offset):
    with open(out_full_path_csv) as f:
        reader = csv.reader(f)
        num_columns = len(next(reader))
        num_lines = 1 + sum(1 for row in reader)
        
# if negative offset go from right
for COL_CAPi in Constants.COL_TEMP_CAP:
    if (COL_CAPi < 0):
        COL_CAPi += num_columns
for COL_TEMP_CAPi in Constants.COL_TEMP_CAP:
    if (COL_TEMP_CAPi < 0):
        COL_TEMP_CAPi += num_columns
for COL_PRXi in Constants.COL_PRX:
    if (COL_PRXi < 0):
        COL_PRXi += num_columns
for COL_TEMP_PRXi in Constants.COL_TEMP_CAP:
    if (COL_TEMP_PRXi < 0):
        COL_TEMP_PRXi += num_columns

# Collect needed temperature columns
COL_TEMP = list(set(Constants.COL_TEMP_CAP + Constants.COL_TEMP_PRX))
COL_TEMP.sort()

# Setup data storage
x = numpy.empty(num_lines)
y_T = numpy.empty((len(COL_TEMP), num_lines)) 
y_CAP = numpy.empty((len(Constants.COL_CAP), num_lines))
dy_CAP = numpy.empty((len(Constants.COL_CAP), num_lines-20))
ddy_CAP = numpy.empty((len(Constants.COL_CAP), num_lines-21))
y_PRX = numpy.empty((len(Constants.COL_PRX), num_lines)) 
dy_PRX = numpy.empty((len(Constants.COL_PRX), num_lines-1)) 
ddy_PRX = numpy.empty((len(Constants.COL_PRX), num_lines-2)) 

# Fill data storage
with open(out_full_path_csv) as csvfile:
    reader = csv.reader(csvfile)
    first_row = next(reader)
    humidity_start = float(first_row[Constants.COL_HUM]) / 100.0
    temp_start = float(first_row[Constants.COL_TEMP_HUM])
    csvfile.seek(0) # reset to first line
    t = 0.0
    for i_row, row in enumerate(reader):
        x[i_row] = t
        # get capacitor values
        for i_y, y_CAPi in enumerate(y_CAP):
            y_CAPi[i_row] = float(row[Constants.COL_CAP[i_y]])
        # get proximity values
        for i_y, y_PRXi in enumerate(y_PRX):
            y_PRXi[i_row] = float(row[Constants.COL_PRX[i_y]])
        # get temperature values
        for i_y, y_Ti in enumerate(y_T):
            y_Ti[i_row] = float(row[COL_TEMP[i_y]])  
        t = t + Constants.PERIOD

# Interpolation
smoothing_CAP = 5000
smoothing_PRX1 = smoothing_CAP
smoothing_PRX2 = 100000000

xnew = numpy.arange(0, num_lines * Constants.PERIOD, 0.2)
y_CAP_coeffs = []
y_PRX_coeffs = []
for i, y_CAPi in enumerate(y_CAP):
    y_CAP_coeffs.append(splrep(x, y_CAPi, s=smoothing_CAP))
for i, y_PRXi in enumerate(y_PRX):
    if i == 0:
        y_PRX_coeffs.append(splrep(x, y_PRXi, s=smoothing_PRX1))
    else:
        y_PRX_coeffs.append(splrep(x, y_PRXi, s=smoothing_PRX2))
        
lal = []
y_PRX = []
# for i, coeffi in enumerate(y_CAP_coeffs):
#     y_CAP.append(splev(x, y_CAP_coeffs[i]))
lal = numpy.convolve(y_CAP[0], numpy.ones(20), "valid") / 20
y_CAP = []
y_CAP.append(lal)
for i, coeffi in enumerate(y_PRX_coeffs):
    y_PRX.append(splev(x, y_PRX_coeffs[i]))

# derivate
for i, y_CAPi in enumerate(y_CAP):
    dy_CAP[i] = diff(y_CAPi) / Constants.PERIOD
for i, y_PRXi in enumerate(y_PRX):
    dy_PRX[i] = diff(y_PRXi) / Constants.PERIOD


# Averaging of derivate (k=10)
# dy_CAPi = numpy.convolve(dy_CAP[0], numpy.ones(20), "valid")/20
# dy_CAP[0][19:] = dy_CAPi
# for dy_CAPi in dy_CAP:
#     for j in range(2,len(dy_CAPi)):
#         dy_CAPi[j] = (dy_CAPi[j] + dy_CAPi[j-1] + dy_CAPi[j-2]) / 3
# for dy_PRXi in dy_PRX:
#     for j in range(3,len(dy_PRXi)):
#         dy_PRXi[j] = (dy_PRXi[j] + dy_PRXi[j-1] + dy_PRXi[j-2]) / 3
        
# derivate²
for i, dy_CAPi in enumerate(dy_CAP):
    ddy_CAP[i] = diff(dy_CAPi) / Constants.PERIOD
for i, dy_PRXi in enumerate(dy_PRX):
    ddy_PRX[i] = diff(dy_PRXi) / Constants.PERIOD
        

# Create needed amount of subplots
num_plots = len(Constants.COL_CAP) + len(Constants.COL_PRX)
fig, ax1 = plt.subplots(num_plots)
                        
# get the respective indices for the temperature to be plotted against
temp_indices = []
for i in Constants.COL_TEMP_CAP:
    try:
        temp_indices.append(COL_TEMP.index(i))
    except ValueError:
        pass
for i in Constants.COL_TEMP_PRX:
    try:
        temp_indices.append(COL_TEMP.index(i))
    except ValueError:
        pass

# Setup temperature graphs
for i_ax1, ax1i in enumerate(ax1):
    ax1i.plot(x, y_T[temp_indices[i_ax1]])
    ax1i.set_xlabel('Time')
    ax1i.set_ylabel('Temperature', color='blue')
    ax1i.tick_params('y', colors='blue')

# Create secondary axis for each temperature vs. time plot
ax2 = []
ax3 = []
ax4 = []
for i_ax1, ax1i in enumerate(ax1):
    ax2.append(ax1i.twinx())
    ax3.append(ax1i.twinx())
    ax4.append(ax1i.twinx())
    
    
# Populate capacitance/light y value graphs
for i_ax2, ax2i in enumerate(ax2):
    if i_ax2 < len(Constants.COL_CAP):
        ax2i.plot(x[19:], y_CAP[i_ax2], 'red')
        ax2i.set_ylabel('Frequency', color='red')
    else:
        ax2i.plot(x, y_PRX[i_ax2 - len(Constants.COL_CAP)], 'red')
        ax2i.set_ylabel('Light value', color='red') 
    ax2i.tick_params('y', colors='red')
    
# Populate capacitance/light dy value graphs
for i_ax3, ax3i in enumerate(ax3):
    if i_ax3 < len(Constants.COL_CAP):
        ax3i.plot(x[20:], dy_CAP[i_ax3], '#ff000060', linestyle='solid')
        ax3i.set_ylim(Constants.DCAP_YLIM[i_ax3])
    else:
        ax3i.plot(x[1:], dy_PRX[i_ax3 - len(Constants.COL_CAP)], '#ff000060', linestyle='solid')
        ax3i.set_ylim(Constants.DPRX_YLIM[i_ax3 - len(Constants.COL_CAP)])
    ax3i.tick_params('y', colors='red')
    ax3i.yaxis.tick_left()
    ax3i.spines['left'].set_position(('axes', 1.0))
    
# Populate capacitance/light ddy value graphs
for i_ax4, ax4i in enumerate(ax4):
    if i_ax4 < len(Constants.COL_CAP):
        ax4i.plot(x[21:], ddy_CAP[i_ax4], '#00000030', linestyle='solid')
        ax4i.set_ylim(Constants.DCAP_YLIM[i_ax4])
    else:
        ax4i.plot(x[2:], ddy_PRX[i_ax4 - len(Constants.COL_CAP)], '#ff000030', linestyle='solid')
        ax4i.set_ylim(Constants.DPRX_YLIM[i_ax4 - len(Constants.COL_CAP)])
    ax4i.tick_params('y', colors='red')
    ax4i.yaxis.tick_left()
    ax4i.spines['left'].set_position(('axes', 1.0))
    
    
# Calculate dew point
def dew_point_magnus(temperature_C: float, relative_humidity:float):
    a = 17.27
    b = 237.7
    gamma = (a * temperature_C) / (b + temperature_C) + numpy.log(relative_humidity)
    dew_point_C = (b * gamma) / (a - gamma)
    return dew_point_C
T_dew_sen = dew_point_magnus(temp_start, humidity_start)
T_dew_ref = dew_point_magnus(temp_start, 0.5736)

def find_indices(array, value):
    indices_exact = []
    indices_between = []
    for i, val in enumerate(array):
        if val == value:
            indices_exact.append(i)
        elif i < len(array)-1 and (array[i] < value < array[i+1] or array[i] > value > array[i+1]):
            indices_between.append([i, i+1])
    return indices_exact, indices_between

for i in range(num_plots):
    indices_exact, indices_between = find_indices(y_T[temp_indices[i]], T_dew_ref)
    for ind in indices_exact:
        ax1[i].scatter(ind * Constants.PERIOD, T_dew_ref, color='blue', label="ja")
    for i_ind, ind in enumerate(indices_between):
        dT_samples = numpy.abs(y_T[temp_indices[i]][ind[1]] - y_T[temp_indices[i]][ind[0]])
        dT_dew = numpy.abs(T_dew_ref - y_T[temp_indices[i]][ind[0]])
        dt = dT_dew / dT_samples * Constants.PERIOD
        ax1[i].scatter(ind[0] * Constants.PERIOD + dt, T_dew_ref, color='blue', label=f'{T_dew_ref}')

for i in range(num_plots):
    indices_exact, indices_between = find_indices(y_T[temp_indices[i]], T_dew_sen)
    for ind in indices_exact:
        ax1[i].scatter(ind * Constants.PERIOD, T_dew_sen, color='#0000ff60', label="ja")
    for i_ind, ind in enumerate(indices_between):
        dT_samples = numpy.abs(y_T[temp_indices[i]][ind[1]] - y_T[temp_indices[i]][ind[0]])
        dT_dew = numpy.abs(T_dew_sen - y_T[temp_indices[i]][ind[0]])
        dt = dT_dew / dT_samples * Constants.PERIOD
        ax1[i].scatter(ind[0] * Constants.PERIOD + dt, T_dew_sen, color='#0000ff60', label=f'{T_dew_sen}')
    

"""
Return indices from arr for which the values are below given threshold
"""
def get_indices_threshold(arr, threshold, below = True):
    # Initialize sublist and indices
    sublists = []
    start = 0
    
    if(below):
        # Check each value against threshold 
        for i in range(1, len(arr)):
            if arr[i] >= threshold and arr[i-1] < threshold: 
                sublists.append([start, i])
                start = i
                
        # Append final subarray
        if arr[-1] < threshold:
            sublists.append([start,-1])
    else:
        for i in range(1, len(arr)):
            if arr[i] <= threshold and arr[i-1] > threshold: 
                sublists.append([start, i])
                start = i
                
        if arr[-1] > threshold:
            sublists.append([start,-1])
        
    return sublists

"""
Get array of arrays from an input array using an indices array
"""
def get_local_maxima(input_array, indices_array):
    # Initialize sublist and indices
    sublists = []
    
    # Check each value against threshold 
    for indices in indices_array:
        i_extrema = argrelextrema(input_array[indices[0]:indices[1]], numpy.greater)
        sublists.append([x + indices[0] for x in i_extrema])
        
    return sublists

cap_avg = stat.fmean(y_CAP[0])
T_avg = stat.fmean(y_T[0])

def get_global_extrema(input_array, average, offset = 0):
    minima_indices = []
    maxima_indices = []

    indices_below = get_indices_threshold(input_array[offset:-1], average, True)
    indices_above = get_indices_threshold(input_array[offset:-1], average, False)    
    
    for ind in indices_below:
        minima_indices.append(offset + ind[0] + numpy.argmin(input_array[ind[0]:ind[1]]))
    for ind in indices_above:
        maxima_indices.append(offset + ind[0] + numpy.argmax(input_array[ind[0]:ind[1]]))
        
    return minima_indices, maxima_indices
       

print("---")
print(humidity_start)
print(T_dew_ref)
print(T_dew_sen)
print("---")

global_Tminima, global_Tmaxima = get_global_extrema(y_T[0], T_avg, 10)
print(global_Tminima, global_Tmaxima)
search_indices = []
for i, minim in enumerate(global_Tminima):
    search_indices.append([global_Tmaxima[i], minim])
dyPeak = 100 + numpy.argmax(dy_CAP[0][search_indices[0][0]+100:search_indices[0][1]])
ddyPeak = numpy.argmax(ddy_CAP[0][search_indices[0][0]:search_indices[0][1]])
print(ddyPeak)
print(dyPeak)
for i in range(ddyPeak, dyPeak):
    print(y_T[temp_indices[0]][i])
# indices = get_indices_threshold(y_CAP[0], 6300)
# print(get_local_maxima(dy_CAP[0], indices))
        
fig.tight_layout()
plt.savefig(out_full_path_png)
plt.show()