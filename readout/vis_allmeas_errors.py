# -*- coding: utf-8 -*-
"""
Visualization of all measurements, possible to visualize all cycles for each
Created on Sun May 12 12:58:27 2024

@author: malte
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
import StyleHelper as SH
SH.update_pyplot_rcParams()

num_sensors = 3
num_meas = 12
num_cycles = 4

sht_dews = []
sht_rhs = []
sensor_dew = [[], [], []]
sensor_rh = [[], [], []]

filename = "meas_output/extract.csv"
with open(filename) as csvfile:
    reader = csv.reader(csvfile)
    for i in range(4):
        next(reader)
    for i in range(3):
        for j in range(4):
            currentrow = next(reader)
            offset = 4
            Tdew_ref = float(currentrow[3])
            RH_ref = float(currentrow[2])
            meas_dew = []
            meas_rh = []
            for c in range(0,4,1):
                cdew1 = -Tdew_ref + float(currentrow[offset + 6 * c + 0])
                cdew2 = -Tdew_ref + float(currentrow[offset + 6 * c + 1])
                crh1 = -RH_ref + float(currentrow[offset + 6 * c + 2])
                crh2 = -RH_ref + float(currentrow[offset + 6 * c + 3])
                meas_dew.append([cdew1, cdew2])
                meas_rh.append([crh1, crh2])
            sensor_dew[0].append(meas_dew)
            sensor_rh[0].append(meas_rh)
            
            meas_dew = []
            meas_rh = []
            for c in range(4,8,1):
                cdew1 = -Tdew_ref + float(currentrow[offset + 6 * c + 0])
                cdew2 = -Tdew_ref + float(currentrow[offset + 6 * c + 1])
                crh1 = -RH_ref + float(currentrow[offset + 6 * c + 2])
                crh2 = -RH_ref + float(currentrow[offset + 6 * c + 3])
                meas_dew.append([cdew1, cdew2])
                meas_rh.append([crh1, crh2])
            sensor_dew[1].append(meas_dew)
            sensor_rh[1].append(meas_rh)
            
            meas_dew = []
            meas_rh = []
            for c in range(8,12,1):
                cdew1 = -Tdew_ref + float(currentrow[offset + 6 * c + 0])
                cdew2 = -Tdew_ref + float(currentrow[offset + 6 * c + 1])
                crh1 = -RH_ref + float(currentrow[offset + 6 * c + 2])
                crh2 = -RH_ref + float(currentrow[offset + 6 * c + 3])
                meas_dew.append([cdew1, cdew2])
                meas_rh.append([crh1, crh2])
            sensor_dew[2].append(meas_dew)
            sensor_rh[2].append(meas_rh)
                
            print(f"{Tdew_ref} {RH_ref} {sensor_rh[0][-1][-1][0]} {sensor_rh[1][-1][-1][1]} {sensor_rh[2][-1][-1][1]}")

            sht_rh = []
            for c in range(-8, -4, 1):
                sht_rh.append(-RH_ref + float(currentrow[c])*100)
            sht_rhs.append(sht_rh)
            sht_dew = []
            for c in range(-4, 0, 1):
                sht_dew.append(-Tdew_ref + float(currentrow[c]))
            sht_dews.append(sht_dew)
    
# prx = [[-0.5,-0.6], [-0.2,-0.3], [-0.15,-0.1], [0.1,-0.02]]
# prx = list(prx for i in range(12))
# labels_T = [list("\\qty{10}{\celsius}\n" for i in range(4)), list("\\qty{25}{\celsius}\n" for i in range(4)),list("\\qty{40}{\celsius}\n" for i in range(4))]
labels_T = [["\\qty{10}{\celsius}\n", "\n", "\n", "\n"], ["\\qty{25}{\celsius}\n", "\n", "\n", "\n"], ["\\qty{40}{\celsius}\n", "\n", "\n", "\n"]]
labels_RH =  list(("\\qty{30}{\%}", "\\qty{50}{\%}", "\\qty{75}{\%}", "\\qty{98}{\%}") for i in range(3))
# labels_RH = 
labels = []
for i in range(3):
    for j in range(4):
        labels.append(labels_T[i][j] + labels_RH[i][j])
print(labels)
# labels = list("\\qty{10}{\celsius}\nA" for i in range(12))
a = [1,4,7,10,6,6,7]
b = [2,3,6,12,7,8,9]
meas = np.arange(0, len(sensor_dew))
colwidth = 0.8
cycles = 4
cyclewidth = colwidth / cycles
cycle_centers = - ((cycles-1) / 2 * cyclewidth) + np.arange(cycles) * cyclewidth
cycledist = 0.04
barwidth = (colwidth - 4 * cycledist) / 4 / 2
w1 = barwidth
w2 = 0.1


# CSV
writelines = [None] * 7
writelines[0] = ',CAP,,,,PRX1,,,,PRX2,,,'
writelines[1] = ',1,,2,,1,,2,,1,,2,'
writelines[2] = ',dew,rh,dew,rh,dew,rh,dew,rh,dew,rh,dew,rh'
writelines[3] = 'min'
writelines[4] = 'max'
writelines[5] = 'avg'
writelines[6] = 'std'
# sensor, poi1/2, dew/rh, meas
values = np.ndarray((num_sensors, 2, 2, num_meas))
# sensor, poi1/2, dew/rh
val_min = np.ndarray((num_sensors, 2, 2))
val_max = np.ndarray((num_sensors, 2, 2))
val_avg = np.ndarray((num_sensors, 2, 2))
val_std = np.ndarray((num_sensors, 2, 2))
for s in range(num_sensors):
    for meas in range(num_meas):
        values[s][0][0][meas] = sensor_dew[s][meas][3][0]
        values[s][0][1][meas] = sensor_rh[s][meas][3][0]
        values[s][1][0][meas] = sensor_dew[s][meas][3][1]
        values[s][1][1][meas] = sensor_rh[s][meas][3][1]
    for i in range(2):
        for j in range(2):
            val_min[s][i][j] = np.min(values[s][i][j])
            val_max[s][i][j] = np.max(values[s][i][j])
            val_avg[s][i][j] = np.mean(values[s][i][j])
            val_std[s][i][j] = np.std(values[s][i][j])
            writelines[3] += (',' + str(val_min[s][i][j]))
            writelines[4] += (',' + str(val_max[s][i][j]))
            writelines[5] += (',' + str(val_avg[s][i][j]))
            writelines[6] += (',' + str(val_std[s][i][j]))
print(writelines)
with open('meas_output/results_accuracy.csv', 'w', newline='') as f:
    writer = csv.writer(f, dialect='excel-tab')
    for line in writelines:
        writer.writerow([line])

# PLOTTING
dew_rh_string = ['T_{Dew}', 'RH']
t_string = ['t', 't']
barlabels = np.full((2, num_sensors, num_meas, num_cycles, 2), None)
for i, dew_rh in enumerate(t_string):
    # barlabels[i][0][0][3][0] = '$\Delta ' + dew_rh + '(max(ddy))$'
    barlabels[i][0][0][3][0] = '$' + dew_rh + '(max(ddy))$'
    barlabels[i][0][0][3][1] = '$' + dew_rh + '(dy=0)$'
    barlabels[i][1][0][3][0] = '$' + dew_rh + '(min(ddy))$'
    barlabels[i][1][0][3][1] = '$' + dew_rh + '(min(dy))$'
    barlabels[i][2][0][3][0] = '$' + dew_rh + '(min(ddy))$'
    barlabels[i][2][0][3][1] = '$' + dew_rh + '(min(ddy))$'

    
    # # BAR
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # fig2 = plt.figure()
    # ax2 = fig2.add_subplot(111)
    # for meas in range(num_meas):
    #     for c in range(num_cycles):
    #         # print(c)
    #         if c == 0 or c == 1:
    #             continue
    #         ax.bar(meas+cycle_centers[c]-w1/2, sensor_dew[s][meas][c][0], w1, color=SH.IES_YELLOW_100, alpha=0.25+c*0.25, label = barlabels[0][s][meas][c][0])
    #         ax.bar(meas+cycle_centers[c]+w1/2, sensor_dew[s][meas][c][1], w1, color=SH.IES_BLUE_100, alpha=0.25+c*0.25, label = barlabels[0][s][meas][c][1])
    #         ax2.bar(meas+cycle_centers[c]-w1/2, sensor_rh[s][meas][c][0], w1, color=SH.IES_YELLOW_100, alpha=0.25+c*0.25, label = barlabels[1][s][meas][c][0])
    #         ax2.bar(meas+cycle_centers[c]+w1/2, sensor_rh[s][meas][c][1], w1, color=SH.IES_BLUE_100, alpha=0.25+c*0.25, label = barlabels[1][s][meas][c][1])
    #         # ax.scatter(meas+cycle_centers[c]-w1/2, sht_dews[meas][c], marker="_", s=7, color=SH.IES_RED_100, alpha=0.25+c*0.25)
    #         # ax.scatter(meas+cycle_centers[c]+w1/2, sht_dews[meas][c], marker="_", s=7, color=SH.IES_RED_100, alpha=0.25+c*0.25)
    #         if meas == 0 and c == num_cycles-1:
    #             ax.scatter(meas+cycle_centers[c]-w1/2, sht_dews[meas][0], marker="_", s=14, color=SH.IES_RED_100, label = 'SHT45, $t=0$')
    #             ax2.scatter(meas+cycle_centers[c]-w1/2, sht_rhs[meas][0], marker="_", s=14, color=SH.IES_RED_100, label = 'SHT45, $t=0$')
    #         ax.scatter(meas+cycle_centers[c]-w1/2, sht_dews[meas][0], marker="_", s=14, color=SH.IES_RED_100)
    #         ax.scatter(meas+cycle_centers[c]+w1/2, sht_dews[meas][0], marker="_", s=14, color=SH.IES_RED_100)
    #         ax2.scatter(meas+cycle_centers[c]-w1/2, sht_rhs[meas][0], marker="_", s=14, color=SH.IES_RED_100)
    #         ax2.scatter(meas+cycle_centers[c]+w1/2, sht_rhs[meas][0], marker="_", s=14, color=SH.IES_RED_100)
    
    # ax.set_xticks(np.arange(num_meas))
    # ax.set_xticklabels(labels)
    # ax.yaxis.grid()
    # ax.set_ylim((-2.5,2.5))
    
    # ax2.set_xticks(np.arange(num_meas))
    # ax2.set_xticklabels(labels)
    # ax2.yaxis.grid()
    # ax2.set_ylim((-12.5,12.5))


    # ax.legend(loc='upper left', ncols=3)
    # ax.set_ylabel('$\Delta ' + dew_rh_string[0] + '$ $[\\unit{\celsius}]$', c=paint_dew)
    # ax2.legend(loc='upper left', ncols=3)
    # ax2.set_ylabel('$\Delta ' + dew_rh_string[1] + '$ $[\%]$', c=paint_rh)
    # # fig.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=4)
    # # fig2.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=4)
    
    # plt.tight_layout()
    # fig.savefig('meas_output/sensor' + str(s) + '_dew.pdf', bbox_inches='tight')
    # # fig2.savefig('meas_output/sensor' + str(s) + '_rh.pdf', bbox_inches='tight')
    # plt.show()
    
    
    
# LINE
ax = plt.subplot()
sensor_poi = [0,1,1]
sensor_paint = [SH.IES_RED_100, SH.IES_BLUE_100, SH.IES_YELLOW_100]
sensor_labels = ['IDE Capacitor', 'VCNL4040', 'VCNL36825T']
ax.axhline(0, color='black')
for s in range(num_sensors):
    
    ax.plot(np.arange(num_meas), list(x[3][sensor_poi[s]] for x in sensor_dew[s]), color=sensor_paint[s], alpha=1, label = sensor_labels[s])
    # ax2.plot(np.arange(num_meas), list(x[3][0] for x in sensor_rh[s]), color=paint_rh, alpha=1, label = barlabels[1][s][meas][3][0])
    ax.scatter(np.arange(num_meas), list(x[3][sensor_poi[s]] for x in sensor_dew[s]), color=sensor_paint[s], alpha=1)
    # ax2.scatter(np.arange(num_meas), list(x[3][0] for x in sensor_rh[s]), color=paint_rh, alpha=1, label = barlabels[1][s][meas][3][0])

    

    ax.set_xticks(np.arange(num_meas))
    ax.set_xticklabels(labels)
    ax.yaxis.grid()
    ax.set_ylim((-1.5,3))

    ax.legend(loc=s+1)
    ax.set_ylabel('$\Delta ' + dew_rh_string[0] + '$ $[\\unit{\celsius}]$')
    # fig.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=4)
    
# plt.tight_layout()
# plt.savefig('meas_output/sensor_alldewmeas.pdf', bbox_inches='tight')
# # fig2.savefig('meas_output/sensor' + str(s) + '_rh.pdf', bbox_inches='tight')
# plt.show()
