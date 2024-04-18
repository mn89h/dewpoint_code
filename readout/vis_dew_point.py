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
import scienceplots
import matplotlib.ticker as ticker
plt.style.use(['science','ieee'])
plt.rcParams.update({'figure.dpi': '100'})
# SH.update_pyplot_rcParams()


class Constants:
    PERIOD          = 0.2
    COL_STATE       = 11
    COL_HUM         = 7 # in old files 5 due to missing T[0]
    COL_TEMP_HUM    = 2
    COL_SENSD       = [-1,-2,-3]
    COL_TEMPD       = [2,1,1]
    SMOOTHING       = [20000,4000,100000000]
    DYLIM           = [[-100, 100], [-80, 80], [-500, 500]]
    DDYLIM          = [[-20, 20], [-50, 50], [-20, 20]]
    AXLABELPAD      = [14.0,14.0,9.0]
    AXLABELS        = ['Frequency (Hz)\nIDE Capacitor', 'Light Intensity\nVCNL4040', 'Light Intensity\nVCNL36825T']
    TCOLOR          = 'b'
    REFCOLOR        = 'b'
    SHTCOLOR        = SH.color(SH.IES_BLUE, 50)
    YCOLORS         = ['r' for i in range(3)]
    DYCOLORS        = [SH.color(SH.IES_RED, 40) for i in range(3)]
    DDYCOLORS       = [SH.color(SH.IES_RED, 35) for i in range(3)]
    INTERPOLATION   = 1
    
    
def get_dew_points(name, plot_graph=False, T_ref=20, rh_ref=50, smoothing=Constants.SMOOTHING):
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
    state_descend = numpy.empty(num_lines)
    y_T = numpy.empty((len(COL_TEMP), num_lines)) 
    y_SENS = numpy.empty((len(Constants.COL_SENSD), num_lines))
    rhs = numpy.empty(num_lines)
    
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
            state = row[Constants.COL_STATE]
            state_descend[i_row] = True if state=="D" else False
            # get capacitor + light values
            for i_y, y_SENSi in enumerate(y_SENS):
                y_SENSi[i_row] = float(row[Constants.COL_SENSD[i_y]])
            # get temperature values
            for i_y, y_Ti in enumerate(y_T):
                y_Ti[i_row] = float(row[COL_TEMP[i_y]])
            rhs[i_row] = float(row[Constants.COL_HUM]) / 100.0
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
    
    
    # Calculate dew point
    def rh_magnus(ref_C: float, dew_C:float):
        a = 17.27
        b = 237.7
        ps_dew = 6.112 * numpy.exp(a*dew_C/(b+dew_C))
        ps_ref = 6.112 * numpy.exp(a*ref_C/(b+ref_C))
        rh = ps_dew / ps_ref * 100
        return rh
    
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
    
    def get_local_minima(input_array, start = 0, stop = -1):
        i_extrema = argrelextrema(input_array[start:stop], numpy.less)
        return [x + start for x in i_extrema]
    
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
    
    def get_next_min1_crossing(input_array):
        return next((i for i, x in enumerate(input_array) if x <= -1), None)
    def get_all_crossing(input_array, th, dirFalling = True, start = 0, stop = -1):
        if dirFalling:
            return (i+1 for i in range(start, start + len(input_array[start:stop])-1) if input_array[i] > th and input_array[i+1] <= th)
        else:
            return (i+1 for i in range(start, start + len(input_array[start:stop])-1) if input_array[i] < th and input_array[i+1] >= th)
    def get_next_crossing(input_array, th, dirFalling = True, start = 0, stop = -1):
        if dirFalling:
            return next((i+1 for i in range(start, start + len(input_array[start:stop])-1) if input_array[i] > th and input_array[i+1] <= th), None)
        else:
            return next((i+1 for i in range(start, start + len(input_array[start:stop])-1) if input_array[i] < th and input_array[i+1] >= th), None)
    def get_next_index(input_array, offset, condition):
        return next((i + offset for i, x in enumerate(input_array[offset:]) if x == condition), None)
    
    search_indices = []
    offset = 0
    for i in range(4):
        descend_start = get_next_index(state_descend, offset, True)
        offset = descend_start
        descend_stop = get_next_index(state_descend, offset, False)
        offset = descend_stop
        
        tmin = descend_start + numpy.argmin(y_T[0][descend_start:descend_stop])
        descend_stop = tmin
        search_indices.append([descend_start,descend_stop])
    print(search_indices)
        
    # above_threshold = []
    # below_threshold = []
    # for i in range(4):
    #     above_threshold.append(list(get_all_crossing(ddy_SENS[0], 0, False, search_indices[i][0], search_indices[i][1])))
    # for i in range(4):
    #     below_threshold.append(list(get_all_crossing(ddy_SENS[0], 0, True, above_threshold[i][-1], search_indices[i][1])))
    # print(above_threshold)
    # print(below_threshold)
    
    off = 0
    detected_dew_range = [[],[],[]]
    det0=[]
    det1=[]
    det2=[]
    # cap
    for ind in search_indices:
        dyThresh = None
        if dy_SENS[0][ind[0]] > -10:
            dyThresh = get_next_crossing(dy_SENS[0], -10, True, ind[0], ind[1])
        # elif dy_SENS[0][ind[0]] > -5:
        #     dyThresh = get_next_crossing(dy_SENS[0], -5, True, ind[0], ind[1])
        if dyThresh == None or dyThresh >= ind[1]-10:
            dyThresh = ind[0]
        dyLow = dyThresh + numpy.argmin(dy_SENS[0][dyThresh:ind[1]])
        ddyPeak = dyThresh + numpy.argmax(ddy_SENS[0][dyThresh:ind[1]])
        # print(ind[0], ind[1], dyThresh, ddyPeak)
        if ddyPeak >= ind[1]-0:
            ddyLow = ddyPeak
        else:
            ddyLow = ddyPeak + numpy.argmin(ddy_SENS[0][ddyPeak:ind[1]])
        
        dy0 = get_next_crossing(dy_SENS[0], 0, False, ddyPeak, ind[1])
        if dy0 == None:
            dy0 = ddyPeak + numpy.argmax(dy_SENS[0][ddyPeak:ind[1]])
        det0.append((ddyPeak, dy0))
    
        
    # prx1
    for ind in search_indices:
        dyThresh = get_next_crossing(dy_SENS[1], -8, True, ind[0], ind[1])
        # print(dyThresh, ind[0], ind[1])
        if dyThresh == None:
            dyThresh = get_next_crossing(dy_SENS[1], -5, True, ind[0], ind[1])
            print("y")
        if dyThresh == None:
            dyThresh = ind[0]
            print("y")
            
        ddyLow = get_local_minima(ddy_SENS[1], dyThresh, ind[1])
        if len(ddyLow[0]) == 0:
            print("check")
            ddyLow = dyThresh + numpy.argmin(ddy_SENS[1][dyThresh:ind[1]])
        else:
            ddyLow = ddyLow[0][0]
            
        dyLow = get_local_minima(dy_SENS[1], dyThresh, ind[1])
        print(dyLow[0][0])
        print(dyThresh + numpy.argmin(dy_SENS[1][dyThresh:ind[1]]))
        if len(dyLow[0]) == 0:
            dyLow = dyThresh + numpy.argmin(dy_SENS[1][dyThresh:ind[1]])
        else:
            dyLow = dyLow[0][0]
            
        det1.append((ddyLow, dyLow))
        
    # prx2
    for ind in search_indices:
        dyThresh = get_next_crossing(dy_SENS[2], -140, True, ind[0], ind[1])
        # print(dyThresh, ind[1])
        if dyThresh == None:
            dyThresh = get_next_crossing(dy_SENS[2], -100, True, ind[0], ind[1])
        if dyThresh == None:
            dyThresh = ind[0]
            
        ddyLow = get_local_minima(ddy_SENS[2], dyThresh, ind[1])
        if len(ddyLow[0]) == 0:
            ddyLow = dyThresh + numpy.argmin(ddy_SENS[2][dyThresh:ind[1]])
        else:
            ddyLow = ddyLow[0][0]
            
        dyLow = get_local_minima(dy_SENS[2], dyThresh, ind[1])
        if len(dyLow[0]) == 0:
            dyLow = dyThresh + numpy.argmin(dy_SENS[2][dyThresh:ind[1]])
        else:
            dyLow = dyLow[0][0]
            
        det2.append((ddyLow, dyLow))
        
    detected_dew_range[0] = det0
    detected_dew_range[1] = det1
    detected_dew_range[2] = det2
    
    # shtT
    shtT = []
    for ind in search_indices:
        shtT.append(y_T[1][ind[0]])
    # shtRH
    shtRH = []
    for ind in search_indices:
        shtRH.append(rhs[ind[0]])
    # shtDew
    shtDew = []
    for ind in search_indices:
        shtDew.append(dew_point_magnus(y_T[1][ind[0]], rhs[ind[0]]))
    
    # for i, det_i in enumerate(detected_dew_range):
    #     if i==0:
    #         print("CAP")
    #     if i==1:
    #         print("PRX1")
    #     if i==2:
    #         print("PRX2")
    #     for det_ij in det_i:
    #         print(y_T[temp_indices[0]][det_ij[0]], y_T[temp_indices[0]][det_ij[1]])
    
    if plot_graph:
        # Create needed amount of subplots
        num_plots = len(Constants.COL_SENSD)
        print_plots = [0,1]
        num_plots = len(print_plots)
        # fig, axT = plt.subplots(num_plots, figsize=SH.set_size(SH.TEXTWIDTH_MASTER * PLOTSIZE, 1, (1,1), 0.5), layout='constrained')
        fig, axT = plt.subplots(num_plots, layout='constrained')
        try:
            iterator = iter(axT)
        except TypeError:
            axT = [axT]
        else:
            pass
        
        # Setup temperature graphs
        for i, axTi in enumerate(axT):
            paint = Constants.TCOLOR
            axTi.plot(x, y_T[temp_indices[i]], c=paint)
            if i == len(axT) - 1: 
                axTi.set_xlabel('Time (s)')
            axTi.set_ylabel('Temperature ($^{\circ}$C)', color=paint)
            axTi.tick_params('y', colors=paint)
            axTi.grid(axis='y')
            axTi.axhline(y = T_dew_ref, linestyle='dashed', color=Constants.REFCOLOR)
            axTi.set_ylim(6,26)
            
            crossings = []
            for ind in search_indices:
                crossings.append(get_next_crossing(y_T[temp_indices[i]], T_dew_ref, True, ind[0], -1))
                
            # for crossing in crossings:
            #     axTi.axvline(x[crossing], linestyle='dashed', color=SH.IES_BLUE_100)
                # csvrow.append(y_SENS[i][crossing])
                # csvrow.append(dy_SENS[i][crossing])
                # csvrow.append(ddy_SENS[i][crossing])
            for det_ij in detected_dew_range[print_plots[i]]:
                axTi.axvline(x=x[det_ij[0]], linestyle='dashed', color=Constants.YCOLORS[0])
                axTi.axvline(x=x[det_ij[1]], linestyle='dashed', color=Constants.YCOLORS[0])
            
            # for abv in above_threshold:
            #     if len(abv) != 0:
            #         axTi.axvline(x=x[abv[-1]])
            
            # for blw in below_threshold:
            #     if len(blw) != 0:
            #         axTi.axvline(x=x[blw[-1]])
               
        # Write Header
        exportfile = 'meas_output/extract.csv'
        if not os.path.isfile(exportfile):
            with open(exportfile, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                row1 = ['MEAS, T_start, RH_ref, T_dew_ref']
                row2 = [',,,']
                row3 = [',,,']
                row4 = [',,,']
                
                for i, sens_dew_ranges in enumerate(detected_dew_range):
                    for j, sens_dew_range_iter in enumerate(sens_dew_ranges):
                        # row1
                        if j == 0:
                            if i == 0:
                                row1.append(',CAP,,,,,')
                            elif i == 1:
                                row1.append(',PRX1,,,,,')
                            elif i == 2:
                                row1.append(',PRX2,,,,,')
                        else:
                            row1.append(',,,,,,')
                        # row2 (Cycle...)
                        row2.append(',Cycle' + str(j) + ',,,,,')
                        # row3 (T, ,i, )
                        row3.append(',T,,RH,,i,')
                        # row4 (POI1,POI2,POI1,POI2,POI1,POI2)
                        if i == 0:
                            row4.append(',dyLow,ddy0,dyLow,ddy0,dyLow,ddy0')
                        else:
                            row4.append(',ddyLow,dyLow,ddyLow,dyLow,ddyLow,dyLow')
                            
                row1.append(',Tref_SHT,,,,RH_SHT,,,,Tdew_SHT,,,')
                row2.append(',0,1,2,3,0,1,2,3,0,1,2,3')
                row3.append(',,,,,,,,,,,,')
                row4.append(',,,,,,,,,,,,')
                writer.writerow([''.join(row1)])
                writer.writerow([''.join(row2)])
                writer.writerow([''.join(row3)])
                writer.writerow([''.join(row4)])
                
        # Write content
        with open(exportfile, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            csvrow = []
            csvrow.append(name)
            csvrow.append(T_ref)
            csvrow.append(rh_ref)
            csvrow.append(T_dew_ref)
            for i, sens_dew_ranges in enumerate(detected_dew_range):
                for sens_dew_range_iter in sens_dew_ranges:
                    x1 = sens_dew_range_iter[0]
                    x2 = sens_dew_range_iter[1]
                    # Dew Temperature
                    dew1 = y_T[temp_indices[i]][x1]
                    dew2 = y_T[temp_indices[i]][x2]
                    csvrow.append(dew1)
                    csvrow.append(dew2)
                    # RH
                    csvrow.append(rh_magnus(T_ref, dew1))
                    csvrow.append(rh_magnus(T_ref, dew2))
                    # indices
                    csvrow.append(x1)
                    csvrow.append(x2)
            for sht_t in shtT:
                csvrow.append(sht_t)
            for sht_rh in shtRH:
                csvrow.append(sht_rh)
            for sht_dew in shtDew:
                csvrow.append(sht_dew)
            
            writer.writerow(csvrow)
        
        # Create secondary axis for each temperature vs. time plot
        axY = []
        axDY = []
        axDDY = []
        # axDDDY = []
        for i_axT, axTi in enumerate(axT):
            axY.append(axTi.twinx())
            # axDY.append(axTi.twinx())
            # axDDY.append(axTi.twinx())
            # axDDDY.append(axTi.twinx())
            
            
        # Populate capacitance/light y value graphs
        for i, s_plot in enumerate(print_plots):
            paint = Constants.YCOLORS[s_plot]
            # paint = Constants.DDYCOLORS[s_plot]
            axY[i].plot(x, y_SENS[s_plot], label = 'a', c=paint)
            axY[i].set_ylabel(Constants.AXLABELS[s_plot], c=paint)
            # axY[i].set_ylabel(Constants.AXLABELS[s_plot], c=paint, labelpad=Constants.AXLABELPAD[s_plot])
            axY[i].tick_params('y', colors=paint)
            # axY[i].set_ylim(5650,7199)
            
        # Populate capacitance/light dy value graphs
        for i, s_plot in enumerate(print_plots):
            paint = Constants.DYCOLORS[s_plot]
            paint = Constants.YCOLORS[s_plot]
            # axDY[i].plot(x, dy_SENS[s_plot], c=paint, linestyle='solid')
            # axDY[i].set_ylim(Constants.DYLIM[s_plot])
            # axDY[i].set(yticklabels=[])
            # axDY[i].tick_params(left=False, right = False)
            # # axDYi.tick_params('y', colors=paint)
            # # axDYi.yaxis.tick_left()
            # # axDYi.spines['left'].set_position(('axes', 1.0))
            # # axDYi.grid()
            
        # Populate capacitance/light ddy value graphs
        for i, s_plot in enumerate(print_plots):
            paint = Constants.DDYCOLORS[s_plot]
            # paint = Constants.YCOLORS[s_plot]
            # axDDY[i].plot(x, ddy_SENS[s_plot], c=paint, linestyle='solid')
            # axDDY[i].set_ylim(Constants.DDYLIM[s_plot])
            # axDDY[i].tick_params('y', colors=paint)
            # axDDY[i].yaxis.tick_left()
            # axDDY[i].spines['left'].set_position(('axes', 1.0))
            # # axDDYi.margins(y=0.5)
            # x0, x1 = Constants.DDYLIM[s_plot]
            # axDDY[i].set_ylim(x0-0.1*(x1-x0), x1+0.1*(x1-x0))
            # # axDDYi.grid()
            
        # fig.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=4)

        # plt.minorticks_off()
        # axT[0].autoscale()
        # axY[0].autoscale()
        fig.set_figwidth(3.3)
        fig.set_figheight(2.5)
        plt.gcf().set_size_inches(3.3, 2.5)
        # plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
        fig.savefig('cap_t25rh50.jpg', dpi=600)
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


enable = 5

csv_files = [
        ['t10rh33.4',10, 33.47, 0.24, [10000, 40000, 5000000], 0], #+
        ['t10rh57.4',10, 57.36, 0.33, [20000, 20000, 2000000], 0], #explain
        ['t10rh75',10, 75.67, 0.22, [320000, 60000, 8000000], 0], #++
        ['t10rh98.2',10, 98.18, 0.76, [20000, 60000, 16000000], 0], 
        ['t25rh32.8',25, 32.78, 0.16, [20000, 60000, 5000000], 0], 
        ['t25rh52.9',25, 52.89, 0.22, [60000, 80000, 26000000], 0], #+ 
        ['t25rh75',25, 75.29, 0.12, [40000, 60000, 14000000], 0], #+
        ['t25rh97.6',25, 97.59, 0.53, [10000, 60000, 64000000], 0], 
        ['t40rh31.6',40, 31.60, 0.13, [27000, 20000, 16000000], 0], # check
        ['t40rh48.4',40, 48.42, 0.37, [100000, 20000, 16000000],0],
        ['t40rh75',40, 74.68, 0.13, [40000, 20000, 16000000], 0], # check
        ['t40rh96.7',40, 96.71, 0.38, [80000, 60000, 16000000],0]]

if enable == -1:
    for file in csv_files:
        file[-1] = 1
else:
    csv_files[enable][-1] = 1

for file in csv_files:
    if file[-1] == True:
        filepath = os.path.join(path, file[0])
        get_dew_points(filepath, plot_graph=True, T_ref=file[1], rh_ref=file[2], smoothing = file[4])
    # input("Press Enter")


