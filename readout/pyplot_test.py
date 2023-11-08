from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from random import random
from time import sleep
import threading

def ReadSensors():
    global x
    global data
    global data2
    while True:
        x += 1 + np.abs(np.random.randn())
        y = np.random.randn()
        if int(random()*2) == 0:
            data.append((x, y))
        if int(random()*2) == 0:
            data2.append((x + np.abs(np.random.randn()), y+1))
        sleep(0.3)

def UpdateFigure(i):
    global data
    global data2
    ax.relim()
    ax.autoscale_view()
    line.set_data(*zip(*data))
    line2.set_data(*zip(*data2))
    print(*zip(*data))


fig, ax = plt.subplots()
x = 0
y = 0
data = deque([(x,y)], maxlen=100)
data2 = deque([(x,y)], maxlen=100)
line, = plt.plot([], c='black')
line2, = plt.plot([], c='red')


# Starts reading sensor in different thread
oThread_ReadSensors = threading.Thread(target = ReadSensors)
oThread_ReadSensors.daemon = True
oThread_ReadSensors.start()

ani = animation.FuncAnimation(fig, UpdateFigure, interval=1000)
plt.show()

input("Hallo")