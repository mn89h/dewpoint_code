import sys
import csv
import datetime
import struct
import serial
import matplotlib.pyplot as plt
import numpy as np
import collections
import threading
from dataclasses import astuple, dataclass
from time import sleep
import pyvisa
import os

# May be needed for pyvisa in order to access visa32.dll (dependent on system and python configuration)
os.add_dll_directory("C:/Program Files/Keysight/IO Libraries Suite/bin")
os.add_dll_directory("C:/Program Files (x86)/Keysight/IO Libraries Suite/bin")


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
        if not always:
            break

def writeCSV(filename : str, always : bool):
    # gain access to data from sensors and from rref written in main thread
    global data
    global rrefs
    # Write data
    # write only single line if always is False, endless loop otherwise
    while always:
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)

            # Create new row
            row = []

            # Fill first column with rref
            while True:
                try:
                    rref = rrefs.popleft();
                except:
                    continue # try again until data is ready
                break # exit loop
            row.append(str(rref))

            # Leave second column blank (used for derived temperature later)
            row.append("")

            # Fill subsequent columns with data from each sensor (sampling time + temperature in °C)
            for device in range(len(data)):
                while True:
                    try:
                        sensordata = data[device].popleft();
                    except:
                        continue
                    break
                row.append(str(sensordata.time))
                row.append(str(sensordata.data))

            # Write row to file
            writer.writerow(row)

# Collect data
def collectData(numSamples):
    global rrefs
    iteration = 0
    while True:
        if iteration >= numSamples and numSamples != 0:
            break 
        rrefs.append(inst.read().rstrip("\r\n").lstrip(" "))
        for device in range(len(data)):
            ReadSensors(False)
        iteration += 1
    print("Finished Sampling. Enter 0 to quit. ")

# UART Connection
comport_board = 'COM14'
print("Waiting for " + comport_board)
while True:
    try:
        s = serial.Serial(comport_board, 1000000)
        break
    except serial.SerialException:
        print("Could not open port. Retrying.")
        sleep(5)
# UART Connection
rm = pyvisa.ResourceManager()
# Open multimeter
inst = rm.open_resource('GPIB0::1::INSTR')
# Reset
# inst.query('*RST')
sleep(1)
# Measurement mode
inst.query('OHMF')

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
input("Devices ready. Press ENTER to continue.")

# Disable malfunctioning sensor
# s.write(str.encode("TOGGLE_TEMP_SENSOR 2 0\r\n"))


# deque for temperature sensor samples
data = np.empty(len(SENSOR_IDS), object)
for i in range(len(data)):
    data[i] = collections.deque([])

# deque for temperature probe samples (Rref)
rrefs = collections.deque([])

# User input for number of samples
while True:
    try:
        numSamples = int(input("Enter the desired number of samples ('0' for continuous sampling): "))
    except:
        continue
    if numSamples < 0:
        continue
    break

# Creation of output file
filename = "measurements_" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".csv"

# Write header:
# "Ref Probe,       , 0     , <Name Sensor0>, 1     , ..."
# "Rref     , Tref  , Time  , Value         , Time  , ..."
with open(filename, 'w', newline='') as file:
    writer = csv.writer(file)
    header = ["Ref Probe", ""]
    for device in range(len(data)):
        header.append(str(device))
        header.append(str(SENSOR_IDS[device].name))
    writer.writerow(header)
    header = ["Rref", "Tref"]
    for device in range(len(data)):
        header.append("Time")
        header.append("Value")
    writer.writerow(header)

# Turn on temperature reading
if numSamples > 0:
    s.write(str.encode("READ_TEMP_SAMPLES " + str(numSamples) + "\r\n"))
else:
    s.write(str.encode("READ_TEMP_ON\r\n"))

# Starts writeCSV in different thread
oThread_writeCSV = threading.Thread(target = writeCSV, args=(filename, True,))
oThread_writeCSV.daemon = True
oThread_writeCSV.start()

# Starts writeCSV in different thread
oThread_collectData = threading.Thread(target = collectData, args=(numSamples,))
oThread_collectData.daemon = True
oThread_collectData.start()

while True:
    try:
        val = int(input("Enter 0 to stop sampling and exit "))
    except:
        continue
    if val == 0:
        break

# STOP
s.write(str.encode("READ_TEMP_SAMPLES 0\r\n"))
s.write(str.encode("READ_TEMP_OFF\r\n"))
s.write(str.encode("RESET\r\n"))

s.flush()
s.close()
