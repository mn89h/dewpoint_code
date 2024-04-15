
import matplotlib as mpl
# Use the pgf backend (must be set before pyplot imported)
PLOTOPTION = 'pdf'
PLOTSIZE = 0.9
if PLOTOPTION == 'pgf':
    mpl.use('pgf')

import csv
import matplotlib.pyplot as plt
import StyleHelper as SH
SH.update_pyplot_rcParams()


x = []
y1 = [] 
y2 = []

with open('pid_from_t10rh33.4.csv') as csvfile:
    reader = csv.reader(csvfile)
    # next(reader) # skip header row
    t = 0.0
    for row in reader:
        x.append(t) 
        y1.append(float(row[1]))  
        y2.append(float(row[0]))
        t = t + 0.2
        
fig, ax = plt.subplots(figsize=SH.set_size(SH.TEXTWIDTH_MASTER * PLOTSIZE), layout='constrained')
ax.plot(x, y2, label='Target', c=SH.IES_BLUE_100)
ax.plot(x, y1, label='Measured', c=SH.IES_RED_100)

ax.set(xlabel='Time [\\unit{\s}]', ylabel='Temperature [\\unit{\celsius}]')
ax.grid()

fig.autofmt_xdate()


ax.margins(0.0, 0.04)
ax.relim()
# update ax.viewLim using the new dataLim
ax.autoscale_view()

plt.legend()

if PLOTOPTION == 'pgf':
    plt.savefig('lolza.pgf', format = 'pgf')
else:
    plt.savefig('pid_offset.pdf', bbox_inches='tight')
    plt.show()