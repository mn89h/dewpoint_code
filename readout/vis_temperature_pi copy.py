import csv
import matplotlib.pyplot as plt
import datetime
from enum import Enum

path = 'C:\Users\malte\Documents\master\measurements\'
filename = 'pid_t0.2_kp20_ki5_kd30'
fileext = '.csv'

class Constants(Enum):
    PERIOD = 0.2
    COL_TEMP = 0
    COL_CAP = 9
    COL_PRX = 8


x = []
y1 = [] 
y2 = []
y3 = []

samples_full_path = os.path.join(path, filename + fileext)
with open(samples_full_path) as csvfile:
    reader = csv.reader(csvfile)
    # next(reader) # skip header row
    t = 0.0
    for row in reader:
        x.append(t) 
        y1.append(float(row[Constants.COL_TEMP]))  
        y2.append(float(row[Constants.COL_CAP]))
        y3.append(float(row[Constants.COL_PRX]))
        t = t + Constants.PERIOD
        

fig, ax1 = plt.subplots(2)

for i in range(2):
    ax1[i].plot(x, y1)
    ax1[i].set_xlabel('Time')
    ax1[i].set_ylabel('Temperature', color='blue')
    ax1[i].tick_params('y', colors='blue')


ax2 = ax1[0].twinx() 
ax2.plot(x, y2, 'r')
ax2.set_ylabel('Frequency', color='red') 
ax2.tick_params('y', colors='red')

ax3 = ax1[1].twinx() 
ax3.plot(x, y3, 'r')
ax3.set_ylabel('Light value', color='red') 
ax3.tick_params('y', colors='red')

fig.tight_layout()
output_full_path = os.path.join(path, filename + '.png')
plt.savefig()
plt.show()