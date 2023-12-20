import csv
import matplotlib.pyplot as plt
from datetime import time, timedelta
from nptime import nptime

def generate_time_sequence(start_time, stop_time, interval_us):
    time_sequence = []
    current_time = start_time
    while current_time <= stop_time:
        time_sequence.append(current_time)
        # current_time += timedelta(microseconds=interval_us)
        current_time += interval_us
    return time_sequence

def plot_columns_with_time(columns, column_indices, start_time, stop_time, interval_minutes):
    # Generate time sequence
    time_sequence = generate_time_sequence(start_time, stop_time, interval_minutes)

    fig, ax = plt.subplots()
    # plt.figure(figsize=(10, 6))

    for idx in column_indices:
        ax.plot(time_sequence[:len(columns[idx])], columns[idx], label=f'Column {idx}')

    
# ax1.tick_params(axis='y', labelcolor=color)
    ax.set_xlabel('Time')
    ax.set_ylabel('Values')
    ax.legend()
    # ax.xticks(rotation=45)
    ax.relim()
    ax.autoscale_view()
    # plt.tight_layout()
    plt.show()

def read_csv_to_transposed_columns(csv_file, start_index, stop_index):
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)

    data = [[float(col) for col in rows] for rows in data[start_index:stop_index]]
    # Transpose the data to separate columns
    columns = list(zip(*data))
    return columns

# Replace 'data.csv' with the path to your CSV file
# Replace [0, 1, 2] with the indices of columns you want to plot
# start_time = datetime(2023, 1, 1, 0, 0)  # Example start time (YYYY, MM, DD, HH, MM)
start_time = 0
# stop_time = datetime(2023, 1, 2, 0, 0)   # Example stop time (YYYY, MM, DD, HH, MM)
stop_time = 900
interval_us = 50

for tempIndex in range(0,22):
    columns = read_csv_to_transposed_columns("output3.csv", tempIndex * 19 + 2, (tempIndex+1) * 19 + 2)
    plot_columns_with_time(columns, [1, 5, 7, 9, 11, 15], start_time, stop_time, interval_us)