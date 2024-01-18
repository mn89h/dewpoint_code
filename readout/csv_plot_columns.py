import csv
import matplotlib.pyplot as plt
from datetime import time, timedelta
from nptime import nptime
from enum import Enum

def generate_time_sequence(start_time, stop_time, interval):
    time_sequence = []
    current_time = start_time
    while current_time <= stop_time:
        time_sequence.append(current_time)
        current_time += interval
    return time_sequence

def plot_columns_with_time(columns : list, column_indices : list, columnNames : list, start_time : int, stop_time : int, interval : int):
    # Generate time sequence
    if stop_time == 0:
        stop_time = start_time + len(columns[0]) * interval
    time_sequence = generate_time_sequence(start_time, stop_time, interval)

    fig, ax = plt.subplots()
    # plt.figure(figsize=(10, 6))

    for idx in column_indices:
        ax.plot(time_sequence[:len(columns[idx])], columns[idx], label=columnNames[idx])

    
# ax1.tick_params(axis='y', labelcolor=color)
    ax.set_xlabel('Time')
    ax.set_ylabel('Values')
    ax.legend()
    # ax.xticks(rotation=45)
    ax.relim()
    ax.autoscale_view()
    # plt.tight_layout()
    plt.show()

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
columnNames = ["Ref Probe","PTD Reference","0","J2:TMP117","1","J2:Si7051","2","J2:AS6221","3","J2:ADT7422","4","J1:Si7051","5","J1:AS6221","6","J1:ADT7422","7","X:HDC1080JS","8","X:SHT31"]
numberOfHeaderRows = 2                              # leave header rows out from plotting

# SEPERATED mode setup
numberOfSeperatedMeasurements = 21  # from 0°C - 60°C (inclusive)
numberOfSamplesPerMeasurement = 19  # 19 samples per measurement

# X axis setup
start_time = 0      # X axis starts from 0
interval_us = 50    # sampling time intervals
stop_time = 0       # auto-detect
#TODO: take time from csv

if (mode == PrintMode.CONTINUOUS):
    columns = read_csv_to_transposed_columns(filename, numberOfHeaderRows, 0)
    plot_columns_with_time(columns, columnsToBePlotted, columnNames, start_time, stop_time, interval_us)
if (mode == PrintMode.SEPERATED):
    for tempIndex in range(0, numberOfSeperatedMeasurements + 1):
        columns = read_csv_to_transposed_columns(filename, tempIndex * numberOfSamplesPerMeasurement + numberOfHeaderRows, (tempIndex+1) * numberOfSamplesPerMeasurement + numberOfHeaderRows)
        plot_columns_with_time(columns, columnsToBePlotted, columnNames, start_time, stop_time, interval_us)