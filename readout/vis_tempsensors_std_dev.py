
import matplotlib as mpl
# Use the pgf backend (must be set before pyplot imported)
PLOTOPTION = 'pdf'
PLOTSIZE = 1
if PLOTOPTION == 'pgf':
    mpl.use('pgf')

import csv
import matplotlib.pyplot as plt
import StyleHelper as SH
from enum import Enum
import numpy as np
SH.update_pyplot_rcParams()


def read_csv_to_transposed_columns(csv_file : str, start_index : int, stop_index: int):
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)

    if stop_index == 0:
        stop_index = len(data)

    data = [[float(col) for col in rows] for rows in data[start_index:stop_index]]
    # Transpose the data to separate columns
    columns = list(zip(*data))
    return columns

class PrintMode(Enum):
    CONTINUOUS = 0
    SEPERATED = 1

filename = "temp_range/output3.csv"

mode = PrintMode.CONTINUOUS                         # CONTINUOUS for single plot, SEPERATED for multiple plots (e.g. multiple temperature ranges in csv)
# columnsToBePlotted = [1, 7, 5, 11, 9, 15, 17, 19]   # columns containing valid temperature data
columnNames = ["Ref Probe","Reference",
               "0","TMP117",
               "1","Si7051",
               "2","AS6221",
               "3","ADT7422",
               "4","Si7051",
               "5","AS6221",
               "6","ADT7422",
               "7","HDC1080JS",
               "8","SHT31"]
numberOfHeaderRows = 2                              # leave header rows out from plotting

# SEPERATED mode setup
mode = PrintMode.SEPERATED                          # CONTINUOUS for single plot, SEPERATED for multiple plots (e.g. multiple temperature ranges in csv)
# columnsToBePlotted = [1, 5, 7, 9, 11, 15, 17, 19]   # columns containing valid temperature data
columnsToBePlotted = [1, 7, 5, 11, 9, 15]   # columns containing valid temperature data
numberOfSeperatedMeasurements = 21  # from 0°C - 60°C (inclusive)
numberOfSamplesPerMeasurement = 19  # 19 samples per measurement

# X axis setup
start_time = 0      # X axis starts from 0
interval_us = 50    # sampling time intervals
stop_time = 0       # auto-detect
#TODO: take time from csv

if (mode == PrintMode.CONTINUOUS):
    columns = read_csv_to_transposed_columns(filename, numberOfHeaderRows, 0)
    
    
    print(len(columns))
    x = np.arange(len(columnsToBePlotted))
    print(len(x))
    # y = np.empty((0))
    # for idx in columnsToBePlotted:
        # np.append(y, np.mean(columns[idx]))
    y = []
    for idx in columnsToBePlotted:
        y.append(np.mean(columns[idx][1:3000]))
    print(y)
    e = []
    for idx in columnsToBePlotted:
        # np.append(y, np.mean(columns[idx]))
        e.append(np.std(columns[idx][1:3000]))
    print(e)
    
    fig, ax = plt.subplots(figsize=SH.set_size(SH.TEXTWIDTH_MASTER, PLOTSIZE, (1,1), 1.0), layout='constrained')
    
    my_xticks = []
    i = 0
    print(len(columnNames))
    for idx in columnsToBePlotted:
        my_xticks.append(columnNames[idx])
        # ugly fix
        # if i == 0:
        #     my_xticks.append(columnNames[idx])
        i += 1
    plt.xticks(x)
    ax.tick_params(axis='x', rotation=45)
    ax.set_xticklabels(my_xticks, ha='right')
    
    ax.errorbar(x, y, e, ls='None', marker='_', capsize=2, capthick=1, c=SH.IES_BLUE_100)
    
    # ax.set_xlabel('Sensor')
    ax.set_ylabel('Temperature [\\unit{\celsius}]')
    
    ax.relim()
    # update ax.viewLim using the new dataLim
    ax.autoscale_view()
    
    # useless
    # plt.subplots_adjust(top=0.925, 
    #                     bottom=0.20, 
    #                     left=0.2, 
    #                     right=0.90, 
    #                     hspace=0.01, 
    #                     wspace=0.01)
    
    
    
if (mode == PrintMode.SEPERATED):
    temp = []
    deviations = []
    for tempIndex in range(0, numberOfSeperatedMeasurements):
        columns = read_csv_to_transposed_columns(filename, tempIndex * numberOfSamplesPerMeasurement + numberOfHeaderRows, (tempIndex+1) * numberOfSamplesPerMeasurement + numberOfHeaderRows)
        # print(columns)
        
        x = np.arange(len(columnsToBePlotted))
        # y = np.empty((0))
        # for idx in columnsToBePlotted:
            # np.append(y, np.mean(columns[idx]))
        y = []
        for idx in columnsToBePlotted:
            y.append(np.mean(columns[idx]))
        # print(y)
        e = []
        for idx in columnsToBePlotted:
            # np.append(y, np.mean(columns[idx]))
            e.append(np.std(columns[idx]))
            
        ref = y[0]
        temp.append(ref)
        dev = []
        for i in range(1, len(y)):
            dev.append(y[i] - ref)
        deviations.append(dev)
        # print(e)
        
    deviations_sensor = list(zip(*deviations))
    print(deviations_sensor)
    
    fig, ax = plt.subplots(figsize=SH.set_size(SH.TEXTWIDTH_MASTER, PLOTSIZE), layout='constrained')
    
    colors = (SH.IES_BLUE_100, SH.IES_RED_100, SH.IES_RED_100, SH.IES_YELLOW_100, SH.IES_YELLOW_100)
    labels = ("AS6221", "Si7051", "", "ADT7422", "")
    for i, deviations_sensorI in enumerate(deviations_sensor):
        if labels[i] != "":
            ax.plot(temp, [x * 1000 for x in deviations_sensorI], c=colors[i], label=labels[i])
        else:
            ax.plot(temp, [x * 1000 for x in deviations_sensorI], c=colors[i])
    
    # ax.set_xlabel('Sensor')
    ax.set_ylabel('Temperature Deviation [\\unit{\milli\celsius}]')
    ax.set_xlabel('Temperature [\\unit{\celsius}]')
    
    ax.relim()
    # update ax.viewLim using the new dataLim
    ax.autoscale_view()
    
plt.legend()
if PLOTOPTION == 'pgf':
    plt.savefig('lolza.pgf', format = 'pgf')
else:
    plt.savefig('tempsensors_dev.pdf', bbox_inches='tight')
    plt.show()
plt.show()
        