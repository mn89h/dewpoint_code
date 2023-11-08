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

@dataclass
class SensorDescriptor:
    name: str
    color: str


SENSOR_IDS = dict([
    (0, SensorDescriptor('TMP117_LOWER', 'red')),
    (1, SensorDescriptor('TMP117_UPPER', 'black')),
])

@dataclass
class SensorData:
    time: int
    data: float
    
    def __iter__(self):
        return iter(self.__dict__.values())

def ReadSensors():
    global data
    while True:
        id = s.read(4)
        id = struct.unpack('I', id)[0] #uint
        print(id)
        time = s.read(8)
        time = struct.unpack('Q', time)[0] #ulong
        print(time)
        value = s.read(8)
        value = struct.unpack('d', value)[0] #double
        print(value)
        new_data = SensorData(time, value)
        data[id].append(new_data)
        print(data[id])
        sleep(0.3)


def UpdateFigure(var):
    global data
    ax1.relim()
    ax1.autoscale_view()
    for i in range(len(SENSOR_IDS)):
        lines[i].set_data(*zip(*data[i]))


s = serial.Serial('COM12', 115200)
fig, ax1 = plt.subplots()


color = 'black'
ax1.set_xlabel('time (s)')
ax1.set_ylabel('T (°C)', color=color)
ax1.tick_params(axis='y', labelcolor=color)
ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
ax2.set_ylabel('Photodiode', color=color)  # we already handled the x-label with ax1
ax2.tick_params(axis='y', labelcolor=color)

data = np.empty(len(SENSOR_IDS), object)
for i in range(len(data)):
    data[i] = collections.deque([SensorData(0,0)], maxlen=10)

lines = np.empty(len(SENSOR_IDS), matplotlib.lines.Line2D)
for i in range(len(lines)):
    lines[i], = ax1.plot([], c=SENSOR_IDS[i].color)


# Starts reading sensor in different thread
oThread_ReadSensors = threading.Thread(target = ReadSensors)
oThread_ReadSensors.daemon = True
oThread_ReadSensors.start()

ani = animation.FuncAnimation(fig, UpdateFigure, interval=1000)
plt.show()
