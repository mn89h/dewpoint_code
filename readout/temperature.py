import struct
import serial
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import collections
import threading
from dataclasses import astuple, dataclass
from time import sleep
from random import random

# 12 colors
num_plots = 12

# http://matplotlib.org/1.2.1/examples/pylab_examples/show_colormaps.html
colormap = plt.cm.gist_ncar
# colormap = plt.cm.gnuplot2
colors = [colormap(i) for i in np.linspace(0, 1, num_plots)]

@dataclass
class SensorDescriptor:
    name: str
    status: int
    color: str


SENSOR_IDS = dict([])

@dataclass
class SensorData:
    time: float
    data: float
    
    def __iter__(self):
        return iter(self.__dict__.values())
    
def GetSensorData():
        id = s.read(4)
        id = struct.unpack('I', id)[0] #uint
        # print("ID: " + str(id))
        time = s.read(4)
        time = (float) (struct.unpack('I', time)[0]) / 1000 #uint
        # print("Time: " + str(time))
        value = s.read(4)
        # print("Raw value: " + str(value))
        value = (float) (struct.unpack('f', value)[0]) #float
        # print("Value: " + str(value))
        new_data = id, SensorData(time, value)
        return new_data

def ReadSensors(always = True):
    global data
    while True:
        id, new_data = GetSensorData()
        data[id].append(new_data)
        # print(data[id])


def UpdateFigure(var):
    global data
    ax1.relim()
    ax1.autoscale_view()
    for i in range(len(SENSOR_IDS)):
        if len(data[i]) > 0:                    # set only lines which can be filled with data (no display otherwise)
            lines[i].set_data(*zip(*data[i]))   # get x (time) and y (temp/light) values 

# UART Connection
comport = 'COM11'
print("Waiting for " + comport)
while True:
    try:
        s = serial.Serial(comport, 115600)
        break
    except serial.SerialException:
        print("Could not open port. Retrying.")
        sleep(5)

# (RE)START
s.write(str.encode("RESET\r\n"))

# Sensor enumeration
s.write(str.encode("LIST_TEMP_SENSORS\r\n"))
while True:
    sensor_string = s.readline()
    sensor_string = sensor_string.decode()
    sensor_string = sensor_string.rstrip("\r\n")
    if sensor_string == "END":
        break
    if not sensor_string.startswith("ID"):
        continue
    fields = sensor_string.split(", ")
    id = int(fields[0].lstrip("ID: "))
    name = fields[1].lstrip("NAME: ")
    status = int(fields[2].lstrip("STATUS: "))
    SENSOR_IDS.update({id: SensorDescriptor(name, status, colors[id])})

print(SENSOR_IDS)
print("Succes. Press ENTER to continue.")
input()

# Disable malfunctioning sensor
s.write(str.encode("TOGGLE_TEMP_SENSOR 2 0\r\n"))

# Turn on temperature reading
s.write(str.encode("READ_TEMP_ON\r\n"))

# Plot setup and line styling
fig, ax1 = plt.subplots()
color = 'black'
ax1.set_xlabel('time (s)')
ax1.set_ylabel('T (°C)', color=color)
ax1.tick_params(axis='y', labelcolor=color)
ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
ax2.set_ylabel('Photodiode', color=color)  # we already handled the x-label with ax1
ax2.tick_params(axis='y', labelcolor=color)

# Collection for data
data = np.empty(len(SENSOR_IDS), object)
for i in range(len(data)):
    data[i] = collections.deque([])

# Add empty lines to ax1 plot
lines = np.empty(len(SENSOR_IDS), matplotlib.lines.Line2D)
for i in range(len(lines)):
    lines[i], = ax1.plot([], c=SENSOR_IDS[i].color, label=str(SENSOR_IDS[i].name))
ax1.legend(loc="upper left")

# Starts reading sensor in different thread
oThread_ReadSensors = threading.Thread(target = ReadSensors)
oThread_ReadSensors.daemon = True
oThread_ReadSensors.start()

# Show plots only after some data is available
while len(data) == 0:
    sleep(1)

ani = animation.FuncAnimation(fig, UpdateFigure, interval=500)
plt.show()
