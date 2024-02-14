import numpy as np

import csv
import matplotlib.pyplot as plt
from enum import Enum

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

filename = "measurements_20231222-150942_processed.csv"
mode = PrintMode.CONTINUOUS                         # CONTINUOUS for single plot, SEPERATED for multiple plots (e.g. multiple temperature ranges in csv)
columnsToBePlotted = [1, 5, 7, 9, 11, 15, 17, 19]   # columns containing valid temperature data
columnNames = ["Ref Probe","Reference","0","J2:TMP117","1","J2:Si7051","2","J2:AS6221","3","J2:ADT7422","4","J1:Si7051","5","J1:AS6221","6","J1:ADT7422","7","X:HDC1080JS","8","X:SHT31"]
numberOfHeaderRows = 2                              # leave header rows out from plotting

# SEPERATED mode setup
# filename = "measurements_20231222-150942_processed.csv"
# mode = PrintMode.SEPERATED                          # CONTINUOUS for single plot, SEPERATED for multiple plots (e.g. multiple temperature ranges in csv)
# columnsToBePlotted = [1, 5, 7, 9, 11, 15, 17, 19]   # columns containing valid temperature data
numberOfSeperatedMeasurements = 21  # from 0°C - 60°C (inclusive)
numberOfSamplesPerMeasurement = 19  # 19 samples per measurement

# X axis setup
start_time = 0      # X axis starts from 0
interval_us = 50    # sampling time intervals
stop_time = 0       # auto-detect
#TODO: take time from csv

if (mode == PrintMode.CONTINUOUS):
    columns = read_csv_to_transposed_columns(filename, numberOfHeaderRows, 0)
if (mode == PrintMode.SEPERATED):
    for tempIndex in range(0, numberOfSeperatedMeasurements + 1):
        columns = read_csv_to_transposed_columns(filename, tempIndex * numberOfSamplesPerMeasurement + numberOfHeaderRows, (tempIndex+1) * numberOfSamplesPerMeasurement + numberOfHeaderRows)

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

fig, ax = plt.subplots()

my_xticks = []
i = 0
for idx in columnsToBePlotted:
    my_xticks.append(columnNames[idx])
    # ugly fix
    if i == 0:
        my_xticks.append(columnNames[idx])
    i += 1
ax.tick_params(axis='x', rotation=25)
ax.set_xticklabels(my_xticks)

ax.errorbar(x, y, e, linestyle='None', marker='^', capsize=3)

ax.set_xlabel('Sensor')
ax.set_ylabel('Temperature [°C]')


plt.subplots_adjust(top=0.925, 
                    bottom=0.20, 
                    left=0.2, 
                    right=0.90, 
                    hspace=0.01, 
                    wspace=0.01)

plt.savefig('foo.png')
plt.savefig('foo.pdf')
plt.show()