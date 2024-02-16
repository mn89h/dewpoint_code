import csv
import os
import matplotlib.pyplot as plt
import datetime
import re
import numpy

path = 'C:/Users/malte/Documents/master/measurements/'
filename_in = 'hum_test3'
fileext_in = '.txt'
filename_out = 'hum_test3 - Kopie'
# filename = 'pid_t0.2_kp20_ki5_kd30'

disable_all_file_preprocessing = True
if (disable_all_file_preprocessing):
    fileext_in = '.csv'
split_header = True
check_whitespace = True
col_as_offset = True


class Constants:
    PERIOD          = 0.2
    COL_HUM         = 6 # in old files 5 due to missing T[0]
    COL_TEMP_HUM    = 2
    COL_CAP         = [-1]
    COL_TEMP_CAP    = [2]
    COL_PRX         = [-2,-3]
    COL_TEMP_PRX    = [1,1]
    

in_full_path = os.path.join(path, filename_in + fileext_in)
out_full_path_csv = os.path.join(path, filename_out + '.csv')
out_full_path_txt = os.path.join(path, filename_out + '.txt')
out_full_path_png = os.path.join(path, filename_out + '.png')

if (~disable_all_file_preprocessing):
    # Split the input file into csv (samples) and txt (info header)
    if (split_header):
        with open(in_full_path, 'r') as infile:
            lines = infile.readlines()
            
        with open(out_full_path_txt, 'w') as outfile1: 
            outfile1.writelines(lines[:3])
            
        with open(out_full_path_csv, 'w') as outfile2:
           outfile2.writelines(lines[3:-1])
    
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
print(f'lol: {COL_TEMP} {Constants.COL_CAP} {Constants.COL_PRX}')

# Setup data storage
x = numpy.empty(num_lines)
y_T = numpy.empty((len(COL_TEMP), num_lines)) 
y_CAP = numpy.empty((len(Constants.COL_CAP), num_lines))
y_PRX = numpy.empty((len(Constants.COL_PRX), num_lines)) 

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
for i_ax1, ax1i in enumerate(ax1):
    ax2.append(ax1i.twinx())
    
# Populate capacitance/light value graphs
for i_ax2, ax2i in enumerate(ax2):
    if i_ax2 < len(Constants.COL_CAP):
        ax2i.plot(x, y_CAP[i_ax2], 'r')
        ax2i.set_ylabel('Frequency', color='red')
    else:
        ax2i.plot(x, y_PRX[i_ax2 - len(Constants.COL_CAP)], 'r')
        ax2i.set_ylabel('Light value', color='red') 
    ax2i.tick_params('y', colors='red')
    
    
# Calculate dew point
def dew_point_magnus(temperature_C: float, relative_humidity:float):
    a = 17.27
    b = 237.7
    gamma = (a * temperature_C) / (b + temperature_C) + numpy.log(relative_humidity)
    dew_point_C = (b * gamma) / (a - gamma)
    return dew_point_C
T_dew_ref = dew_point_magnus(temp_start, humidity_start)

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
    
        
fig.tight_layout()
plt.savefig(out_full_path_png)
plt.show()