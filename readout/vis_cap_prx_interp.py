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
import StyleHelper as SH

path = 'C:/Users/malte/Documents/master/measurements/thesis/'
# filename_in = 'hum_test38'
filename_in = 't10rh57.4'
fileext_in = '.txt'
filename_out = filename_in
# filename = 'pid_t0.2_kp20_ki5_kd30'

in_file = path + filename_in

disable_all_file_preprocessing = False
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
    COL_SENSD       = [-1,-2,-3]
    COL_TEMPD       = [2,1,1]
    SMOOTHING       = [2000,2000,100000000]
    DYLIM           = [[-100, 30], [-50, 50], [-300, 300]]
    LABELS          = ['Frequency', 'Light Value', 'Light Value']
    TCOLOR          = SH.IES_BLUE_100
    REFCOLOR        = SH.color(SH.IES_BLUE, 80)
    SHTCOLOR        = SH.color(SH.IES_BLUE, 50)
    YCOLORS         = [SH.IES_RED_100 for i in range(3)]
    DYCOLORS        = [SH.color(SH.IES_YELLOW, 50) for i in range(3)]
    DDYCOLORS       = [SH.color(SH.IES_YELLOW, 30) for i in range(3)]
    
print(Constants.YCOLORS)
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
for COL_Si in Constants.COL_SENSD:
    if (COL_Si < 0):
        COL_Si += num_columns
for COL_Ti in Constants.COL_TEMPD:
    if (COL_Ti < 0):
        COL_Ti += num_columns

# Collect needed temperature columns
COL_TEMP = list(set(Constants.COL_TEMPD))
COL_TEMP.sort()

# Setup data storage
x = numpy.empty(num_lines)
y_T = numpy.empty((len(COL_TEMP), num_lines)) 
y_SENS = numpy.empty((len(Constants.COL_SENSD), num_lines))
dy_SENS = numpy.empty((len(Constants.COL_SENSD), num_lines-1))
ddy_SENS = numpy.empty((len(Constants.COL_SENSD), num_lines-2)) 

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
        # get capacitor + light values
        for i_y, y_SENSi in enumerate(y_SENS):
            y_SENSi[i_row] = float(row[Constants.COL_SENSD[i_y]])
        # get temperature values
        for i_y, y_Ti in enumerate(y_T):
            y_Ti[i_row] = float(row[COL_TEMP[i_y]])  
        t = t + Constants.PERIOD

# Interpolation
xnew = numpy.arange(0, num_lines * Constants.PERIOD, 0.2)
y_SENS_coeffs = []
for i, y_SENSi in enumerate(y_SENS):
    # y_CAP_coeffs.append(splrep(x, y_CAPi, w=numpy.ones(len(y_CAPi))))
    y_SENS_coeffs.append(splrep(x, y_SENSi, s=Constants.SMOOTHING[i]))
        
y_SENS = []
for i, coeffi in enumerate(y_SENS_coeffs):
    y_SENS.append(splev(x, coeffi))

# derivate
for i, y_SENSi in enumerate(y_SENS):
    dy_SENS[i] = diff(y_SENSi) / Constants.PERIOD


# Averaging of derivate (k=10)
# dy_CAPi = numpy.convolve(dy_CAP[0], numpy.ones(20), "valid")/20
# dy_CAP[0][19:] = dy_CAPi
        
# derivate²
for i, dy_SENSi in enumerate(dy_SENS):
    ddy_SENS[i] = diff(dy_SENSi) / Constants.PERIOD
        

# Create needed amount of subplots
num_plots = len(Constants.COL_SENSD)
fig, axT = plt.subplots(num_plots)
                        
# get the respective indices for the temperature to be plotted against
temp_indices = []
for i in Constants.COL_TEMPD:
    try:
        temp_indices.append(COL_TEMP.index(i))
    except ValueError:
        pass

# Setup temperature graphs
for i, axTi in enumerate(axT):
    paint = Constants.TCOLOR
    axTi.plot(x, y_T[temp_indices[i]], c=paint)
    if i == len(axT) - 1: 
        axTi.set_xlabel('Time')
    axTi.set_ylabel('Temperature', color=paint)
    axTi.tick_params('y', colors=paint)

# Create secondary axis for each temperature vs. time plot
axY = []
axDY = []
axDDY = []
for i_axT, axTi in enumerate(axT):
    axY.append(axTi.twinx())
    axDY.append(axTi.twinx())
    axDDY.append(axTi.twinx())
    
    
# Populate capacitance/light y value graphs
for i, axYi in enumerate(axY):
    paint = Constants.YCOLORS[i]
    axYi.plot(x, y_SENS[i], c=paint)
    axYi.set_ylabel(Constants.LABELS[i], c=paint)
    axYi.tick_params('y', colors=paint)
    
# Populate capacitance/light dy value graphs
for i, axDYi in enumerate(axDY):
    paint = Constants.DYCOLORS[i]
    axDYi.plot(x[1:], dy_SENS[i], c=paint, linestyle='solid')
    axDYi.set_ylim(Constants.DYLIM[i])
    axDYi.tick_params('y', colors=paint)
    axDYi.yaxis.tick_left()
    axDYi.spines['left'].set_position(('axes', 1.0))
    axDYi.grid()
    
# Populate capacitance/light ddy value graphs
for i, axDDYi in enumerate(axDDY):
    paint = Constants.DDYCOLORS[i]
    axDDYi.plot(x[2:], ddy_SENS[i], c=paint, linestyle='solid')
    axDDYi.set_ylim(Constants.DYLIM[i])
    axDDYi.tick_params('y', colors=paint)
    axDDYi.yaxis.tick_left()
    axDDYi.spines['left'].set_position(('axes', 1.0))
    axDDYi.grid()
    
    
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

paint = Constants.REFCOLOR
for i in range(num_plots):
    indices_exact, indices_between = find_indices(y_T[temp_indices[i]], T_dew_ref)
    for ind in indices_exact:
        axT[i].scatter(ind * Constants.PERIOD, T_dew_ref, c=paint, label="ja")
    for i_ind, ind in enumerate(indices_between):
        dT_samples = numpy.abs(y_T[temp_indices[i]][ind[1]] - y_T[temp_indices[i]][ind[0]])
        dT_dew = numpy.abs(T_dew_ref - y_T[temp_indices[i]][ind[0]])
        dt = dT_dew / dT_samples * Constants.PERIOD
        axT[i].scatter(ind[0] * Constants.PERIOD + dt, T_dew_ref, c=paint, label=f'{T_dew_ref}')

paint = Constants.REFCOLOR
for i in range(num_plots):
    indices_exact, indices_between = find_indices(y_T[temp_indices[i]], T_dew_sen)
    for ind in indices_exact:
        axT[i].scatter(ind * Constants.PERIOD, T_dew_sen, c=paint, label="ja")
    for i_ind, ind in enumerate(indices_between):
        dT_samples = numpy.abs(y_T[temp_indices[i]][ind[1]] - y_T[temp_indices[i]][ind[0]])
        dT_dew = numpy.abs(T_dew_sen - y_T[temp_indices[i]][ind[0]])
        dt = dT_dew / dT_samples * Constants.PERIOD
        axT[i].scatter(ind[0] * Constants.PERIOD + dt, T_dew_sen, c=paint, label=f'{T_dew_sen}')
    

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

cap_avg = stat.fmean(y_SENS[0])
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
print(global_Tmaxima, global_Tminima)
search_indices = []
for i in range(min(len(global_Tminima), len(global_Tmaxima))):
    search_indices.append([global_Tmaxima[i], global_Tminima[i]])
off = 50
for ind in search_indices:
    dyPeak = ind[0] + off + numpy.argmin(dy_SENS[0][ind[0]+off:ind[1]])
    ddyPeak = ind[0] + off + numpy.argmax(ddy_SENS[0][ind[0]+off:ind[1]])
    print(dyPeak, ddyPeak)
    print(y_T[temp_indices[0]][dyPeak], y_T[temp_indices[0]][ddyPeak])
    # for i in range(ddyPeak, dyPeak):
    #     print(y_T[temp_indices[0]][i])
# indices = get_indices_threshold(y_CAP[0], 6300)
# print(get_local_maxima(dy_CAP[0], indices))
        
fig.tight_layout()
# plt.savefig(out_full_path_png)
plt.show()