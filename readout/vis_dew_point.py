import matplotlib as mpl
# Use the pgf backend (must be set before pyplot imported)
PLOTOPTION = 'pdf'
PLOTSIZE = 1
if PLOTOPTION == 'pgf':
    mpl.use('pgf')


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
SH.update_pyplot_rcParams()


class Constants:
    PERIOD          = 0.2
    COL_HUM         = 7 # in old files 5 due to missing T[0]
    COL_TEMP_HUM    = 2
    COL_SENSD       = [-1,-2,-3]
    COL_TEMPD       = [2,1,1]
    SMOOTHING       = [20000,4000,100000000]
    DYLIM           = [[-100, 100], [-500, 500], [-5000, 5000]]
    DDYLIM          = [[-10, 10], [-50, 50], [-200, 200]]
    AXLABELPAD      = [14.0,14.0,9.0]
    AXLABELS        = ['Frequency  [\\unit{\Hz}]\nIDE Capacitor', 'Light Intensity\nVCNL4040', 'Light Intensity\nVCNL36825T']
    TCOLOR          = SH.IES_BLUE_100
    REFCOLOR        = SH.color(SH.IES_BLUE, 80)
    SHTCOLOR        = SH.color(SH.IES_BLUE, 50)
    YCOLORS         = [SH.IES_RED_100 for i in range(3)]
    DYCOLORS        = [SH.color(SH.IES_RED, 40) for i in range(3)]
    DDYCOLORS       = [SH.color(SH.IES_RED, 20) for i in range(3)]
    INTERPOLATION   = 1
    
    
def get_dew_points(name, plot_graph=False, rh_ref=50, smoothing=Constants.SMOOTHING):
    # Count lines & columns
    in_full_path_csv = name + ".csv"
    col_as_offset = True
    if (col_as_offset):
        with open(in_full_path_csv) as f:
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
    
    # Fill data storage
    with open(in_full_path_csv) as csvfile:
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
    xnew = numpy.arange(0, num_lines * Constants.PERIOD, 0.2 / Constants.INTERPOLATION)
    
    y_SENS_coeffs = []
    dy_SENS_coeffs = []
    ddy_SENS_coeffs = []
    for i, y_SENSi in enumerate(y_SENS):
        y_coeff = splrep(x, y_SENSi, s=smoothing[i])
        # y_coeff = splrep(x, y_CAPi, w=numpy.ones(len(y_CAPi)))
        dy_coeff = BSpline(*y_coeff).derivative()
        print(len(x), len(y_SENSi))
        print(len(y_coeff[0]))
        ddy_coeff = dy_coeff.derivative()
        y_SENS_coeffs.append(y_coeff)
        dy_SENS_coeffs.append(dy_coeff)
        ddy_SENS_coeffs.append(ddy_coeff)
            
    if Constants.INTERPOLATION > 1:    
        y_T_coeffs = []
        for i, y_Ti in enumerate(y_T):
            y_T_coeffs.append(splrep(x, y_Ti))
    
        x = xnew
        y_T = []
        for i, coeffi in enumerate(y_T_coeffs):
            y_T.append(splev(x, coeffi))
    
    y_SENS = []
    for i, coeffi in enumerate(y_SENS_coeffs):
        y_SENS.append(splev(x, coeffi))
    
    dy_SENS = numpy.empty((len(Constants.COL_SENSD), len(y_SENS[0])))
    ddy_SENS = numpy.empty((len(Constants.COL_SENSD), len(y_SENS[0]))) 
    dddy_SENS = numpy.empty((len(Constants.COL_SENSD), len(y_SENS[0]))) 
    # derivate
    for i, dy_SENS_coeff in enumerate(dy_SENS_coeffs):
        dy_SENS[i] = splev(x, dy_SENS_coeff)
        # dy_SENS[i] = diff(y_SENSi) / (Constants.PERIOD / Constants.INTERPOLATION)
        # dy_SENS[i] = diff(y_SENSi) / (1 / Constants.INTERPOLATION)
    
    
    # Averaging of derivate (k=10)
    # dy_CAPi = numpy.convolve(dy_CAP[0], numpy.ones(20), "valid")/20
    # dy_CAP[0][19:] = dy_CAPi
            
    # derivate²
    for i, ddy_SENS_coeff in enumerate(ddy_SENS_coeffs):
        ddy_SENS[i] = splev(x, ddy_SENS_coeff)
        # ddy_SENS[i] = diff(dy_SENSi) / (Constants.PERIOD / Constants.INTERPOLATION)
        # ddy_SENS[i] = diff(dy_SENSi) / (1 / Constants.INTERPOLATION)
    # # derivate³
    # for i, ddy_SENSi in enumerate(ddy_SENS):
    #     dddy_SENS[i] = diff(ddy_SENSi) / (0.2 / Constants.INTERPOLATION)
            
                            
    # get the respective indices for the temperature to be plotted against
    temp_indices = []
    for i in Constants.COL_TEMPD:
        try:
            temp_indices.append(COL_TEMP.index(i))
        except ValueError:
            pass


    # Calculate dew point
    def dew_point_magnus(temperature_C: float, relative_humidity:float):
        a = 17.27
        b = 237.7
        gamma = (a * temperature_C) / (b + temperature_C) + numpy.log(relative_humidity)
        dew_point_C = (b * gamma) / (a - gamma)
        return dew_point_C
    
    def find_indices(array, value):
        indices_exact = []
        indices_between = []
        for i, val in enumerate(array):
            if val == value:
                indices_exact.append(i)
            elif i < len(array)-1 and (array[i] < value < array[i+1] or array[i] > value > array[i+1]):
                indices_between.append([i, i+1])
        return indices_exact, indices_between
    
    
    T_dew_sen = dew_point_magnus(temp_start, humidity_start)
    T_dew_ref = dew_point_magnus(temp_start, rh_ref/100)
    
    cap_avg = stat.fmean(y_SENS[0])
    T_avg = stat.fmean(y_T[0])
    C_avg = stat.fmean(y_SENS[0])
    
    print("---")
    print(humidity_start)
    print(T_dew_ref)
    print(T_dew_sen)
    print("---")
    
    
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
    
    def get_next_zero_crossing(input_array):
        return next((i for i, x in enumerate(input_array) if x <= -1), None)
           
    
    global_Cminima, global_Cmaxima = get_global_extrema(y_SENS[0], C_avg, 10)
    if len(global_Cmaxima) > len(global_Cminima) and global_Cminima[-1] < global_Cmaxima[-1]:
        global_Cminima.append(len(y_SENS[0]))
    
    # detect next maxima for search interval
    next_descends = []
    for maxT in global_Cmaxima:
        desc_index = get_next_zero_crossing(dy_SENS[0][maxT:])
        if desc_index is None:
            desc_index = len(dy_SENS[0])
        else:
            desc_index += maxT
        next_descends.append(desc_index)
        
    # detect next minima for search interval
    T_minima = []
    for i in range(len(next_descends)):
        if i < len(next_descends) - 1:
            T_minima.append(next_descends[i] + numpy.argmin(y_T[0][next_descends[i]:next_descends[i+1]]))
        else:
            T_minima.append(next_descends[i] + numpy.argmin(y_T[0][next_descends[i]:]))
    print(next_descends, T_minima)
    
    # append maxima and minima
    search_indices = []
    for i in range(min(len(next_descends), len(T_minima))):
        search_indices.append([next_descends[i], T_minima[i]])

    off = 0
    detected_dew_range = [[],[],[]]
    det0=[]
    det1=[]
    det2=[]
    # cap
    for ind in search_indices:
        dyLow = ind[0] + off + numpy.argmin(dy_SENS[0][ind[0]+off:ind[1]])
        ddyPeak = ind[0] + off + numpy.argmax(ddy_SENS[0][ind[0]+off:ind[1]])
        ddyLow = ddyPeak + off + numpy.argmin(ddy_SENS[0][ddyPeak+off:ind[1]])
        
        det0.append((ddyPeak, ddyPeak+10))
        off = 0
        
    # prx1
    for ind in search_indices:
        dyLow = ind[0] + off + numpy.argmin(dy_SENS[1][ind[0]+off:ind[1]])
        # ddyPeak = dyLow + numpy.argmax(ddy_SENS[1][dyLow:ind[1]])
        ddyPeak = ind[0] + off + numpy.argmax(ddy_SENS[1][ind[0]+off:ind[1]])
        off = 5
        ddyLow = ind[0] + off + numpy.argmin(ddy_SENS[1][ind[0]+off:ddyPeak])
        
        # ddyPeak = ind[0] + off + numpy.argmax(ddy_SENS[1][ind[0]+off:ind[1]])
        # ddyLow = ddyPeak + off + numpy.argmin(ddy_SENS[1][ddyPeak+off:ind[1]])
        
        det1.append((ddyLow, ddyLow+10))
        off = 0
        
    # prx2
    for ind in search_indices:
        off = 20
        dyLow = ind[0] + off + numpy.argmin(dy_SENS[2][ind[0]+off:ind[1]])
        ddyPeak = ind[0] + off + numpy.argmax(ddy_SENS[2][ind[0] + off:ind[1]])
        ddyLow = ind[0] + off + numpy.argmin(ddy_SENS[2][ind[0]+off:ddyPeak])
        
        # ddyPeak = ind[0] + off + numpy.argmax(ddy_SENS[1][ind[0]+off:ind[1]])
        # ddyLow = ddyPeak + off + numpy.argmin(ddy_SENS[1][ddyPeak+off:ind[1]])
        
        dew_start = ddyLow
        dew_end = ddyLow+10
        if dew_end > len(y_SENS[2]):
            dew_end = len(y_SENS[2]) - 1
        det2.append((dew_start, dew_end))
        off = 0
        detected_dew_range.append(det0)
        detected_dew_range.append(det1)
        detected_dew_range.append(det2)
    print(det1)
    for i, det_i in enumerate(detected_dew_range):
        if i==0:
            print("CAP")
        if i==1:
            print("PRX1")
        if i==2:
            print("PRX2")
        # for det_ij in det_i:
        #     print(y_T[temp_indices[0]][det_ij[0]], y_T[temp_indices[0]][det_ij[1]])
    
    if plot_graph:
        # Create needed amount of subplots
        num_plots = len(Constants.COL_SENSD)
        fig, axT = plt.subplots(num_plots, figsize=SH.set_size(SH.TEXTWIDTH_MASTER * PLOTSIZE, 1, (1,1), 1), layout='constrained')
    
        # Setup temperature graphs
        for i, axTi in enumerate(axT):
            paint = Constants.TCOLOR
            axTi.plot(x, y_T[temp_indices[i]], c=paint)
            if i == len(axT) - 1: 
                axTi.set_xlabel('Time  [\\unit{\s}]')
            axTi.set_ylabel('Temperature [\\unit{\celsius}]', color=paint)
            axTi.tick_params('y', colors=paint)
            axTi.grid(axis='y')
            axTi.axhline(y = T_dew_ref, linestyle='dashed')
            
            for det_ij in detected_dew_range[i]:
                axTi.axvline(x=x[det_ij[0]], linestyle='dashed', color=Constants.DDYCOLORS[0])
                axTi.axvline(x=x[det_ij[1]], linestyle='dashed', color=Constants.DDYCOLORS[0])
        
        # Create secondary axis for each temperature vs. time plot
        axY = []
        axDY = []
        axDDY = []
        # axDDDY = []
        for i_axT, axTi in enumerate(axT):
            axY.append(axTi.twinx())
            axDY.append(axTi.twinx())
            axDDY.append(axTi.twinx())
            # axDDDY.append(axTi.twinx())
            
            
        # Populate capacitance/light y value graphs
        for i, axYi in enumerate(axY):
            paint = Constants.YCOLORS[i]
            axYi.plot(x, y_SENS[i], label = 'a', c=paint)
            axYi.set_ylabel(Constants.AXLABELS[i], c=paint, labelpad=Constants.AXLABELPAD[i])
            axYi.tick_params('y', colors=paint)
            
        # Populate capacitance/light dy value graphs
        for i, axDYi in enumerate(axDY):
            paint = Constants.DYCOLORS[i]
            axDYi.plot(x, dy_SENS[i], c=paint, linestyle='solid')
            axDYi.set_ylim(Constants.DYLIM[i])
            axDYi.set(yticklabels=[])
            axDYi.tick_params(left=False, right = False)
            # axDYi.tick_params('y', colors=paint)
            # axDYi.yaxis.tick_left()
            # axDYi.spines['left'].set_position(('axes', 1.0))
            # axDYi.grid()
            
        # Populate capacitance/light ddy value graphs
        for i, axDDYi in enumerate(axDDY):
            paint = Constants.DDYCOLORS[i]
            axDDYi.plot(x, ddy_SENS[i], c=paint, linestyle='solid')
            axDDYi.set_ylim(Constants.DDYLIM[i])
            axDDYi.tick_params('y', colors=paint)
            axDDYi.yaxis.tick_left()
            axDDYi.spines['left'].set_position(('axes', 1.0))
            # axDDYi.margins(y=0.5)
            x0, x1 = Constants.DDYLIM[i]
            axDDYi.set_ylim(x0-0.1*(x1-x0), x1+0.1*(x1-x0))
            # axDDYi.grid()
            
        # fig.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=4)
            
    fig.tight_layout()
    # plt.savefig(out_full_path_png)
    if PLOTOPTION == 'pgf':
        plt.savefig('lolza.pgf', format = 'pgf')
    else:
        plt.savefig(name + '.pdf', bbox_inches='tight')
        plt.show()



path = 'C:/Users/malte/Documents/master/measurements/thesis/'

# # Initialize an empty array to store the paths of CSV files
# csv_files = []

# # Loop through the files in the folder
# for file_name in os.listdir(path):
#     if file_name.endswith('.csv'):
#         file_path = os.path.join(path, file_name)
#         csv_files.append(file_name)
# print(csv_files)



csv_files = [
        ['t10rh33.4',10, 33.47, 0.24, [10000, 40000, 5000000], 0], #+
        ['t10rh57.4',10, 57.36, 0.33, [20000, 20000, 2000000], 0], #explain
        ['t10rh75',10, 75.67, 0.22, [320000, 60000, 8000000], 0], #++
        ['t10rh98.2',10, 98.18, 0.76, [20000, 60000, 16000000], 0], 
        ['t25rh32.8',25, 32.78, 0.16, [20000, 60000, 5000000], 0], 
        ['t25rh52.9',25, 52.89, 0.22, [60000, 80000, 26000000], 1], #+ 
        ['t25rh75',25, 75.29, 0.12, [40000, 60000, 14000000], 0], #+
        ['t25rh97.6',25, 97.59, 0.53, [10000, 60000, 64000000], 0], 
        ['t40rh31.6',40, 31.60, 0.13, [27000, 20000, 16000000], 0], # check
        ['t40rh48.4',40, 48.42, 0.37, [100000, 20000, 16000000],0],
        ['t40rh75',40, 74.68, 0.13, [40000, 20000, 16000000], 0], # check
        ['t40rh96.7',40, 96.71, 0.38, [80000, 60000, 16000000],0]]

for file in csv_files:
    if file[-1] == True:
        filepath = os.path.join(path, file[0])
        get_dew_points(filepath, plot_graph=True, rh_ref=file[2], smoothing = file[4])
    # input("Press Enter")


