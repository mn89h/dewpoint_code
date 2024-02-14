import csv
import matplotlib.pyplot as plt
import datetime

x = []
y1 = [] 
y2 = []

with open('test3_pid.csv') as csvfile:
    reader = csv.reader(csvfile)
    # next(reader) # skip header row
    t = 0.0
    for row in reader:
        x.append(t) 
        y1.append(float(row[9]))  
        # y2.append(float(row[1]))
        t = t + 0.2
        
fig, ax = plt.subplots()
ax.plot(x, y1, label='Actual')
# ax.plot(x, y2, label='Target')

ax.set(xlabel='Time', ylabel='Temperature (C)',
       title='Temperature over Time')
ax.grid()

fig.autofmt_xdate()
plt.legend()
plt.show()  