import numpy as np

ENHANCEMENT_FACTOR = 1.005 # for air pressure = 1000 hPa
K1w = -6096.9385    # * T90^-1
K2w = 16.635794     #
K3w = -2.711193E-2  # * T90
K4w = 1.673952E-5   # * T90^2
K5w = 2.433502      # * ln(T90)
K1i = -6024.5282    # * T90^-1
K2i = 24.721994     #
K3i = 1.0613868E-2  # * T90
K4i = -1.3198825E-5 # * T90^2
K5i = -0.49382577   # * ln(T90)

T0 = 273.15
Tref = 18.8 + T0
Tdew = 11.55 + T0
Tdew_off = 15.2 + T0

PSref = ENHANCEMENT_FACTOR * np.exp(K1w/Tref + K2w + K3w*Tref + K4w*Tref*Tref + K5w*np.log(Tref))
PSdew = ENHANCEMENT_FACTOR * np.exp(K1w/Tdew + K2w + K3w*Tdew + K4w*Tdew*Tdew + K5w*np.log(Tdew))
PSdew_off = ENHANCEMENT_FACTOR * np.exp(K1w/Tdew_off + K2w + K3w*Tdew_off + K4w*Tdew_off*Tdew_off + K5w*np.log(Tdew_off))

H_real = PSdew/PSref
H_off = PSdew_off/PSref

# print(H_real*100)
# print(H_off*100)
# print(np.abs(H_off-H_real)*100)

csv_files = [
        ['t10rh33.4.csv',10, 33.47, 0.24, [10000, 40000, 5000000], 0], #+
        ['t10rh57.4.csv',10, 57.36, 0.33, [20000, 20000, 2000000], 0], #explain
        ['t10rh75.csv',10, 75.67, 0.22, [320000, 60000, 8000000], 1], #++
        ['t10rh98.2.csv',10, 98.18, 0.76, [20000, 60000, 16000000], 0], 
        ['t25rh32.8.csv',25, 32.78, 0.16, [20000, 60000, 5000000], 0], 
        ['t25rh52.9.csv',25, 52.89, 0.22, [60000, 80000, 26000000], 0], #+ 
        ['t25rh75.csv',25, 75.29, 0.12, [40000, 60000, 14000000], 0], #+
        ['t25rh97.6.csv',25, 97.59, 0.53, [10000, 60000, 64000000], 0], 
        ['t40rh31.6.csv',40, 31.60, 0.13, [27000, 20000, 16000000], 0], # check
        ['t40rh48.4.csv',40, 48.42, 0.37, [100000, 20000, 16000000], 0],
        ['t40rh75.csv',40, 74.68, 0.13, [40000, 20000, 16000000], 0], # check
        ['t40rh96.7.csv',40, 96.71, 0.38, [80000, 60000, 16000000], 0]]

t = [x[1] for x in csv_files]
tK = [x[1]+273.15 for x in csv_files]
rh = [x[2]/100 for x in csv_files]
rh_dev = [x[3]/100 for x in csv_files]

Rw = 461.5
def vapor_pressure_ice(T):
    return ENHANCEMENT_FACTOR * np.exp(K1i/T + K2i + K3i*T + K4i*T*T + K5i*np.log(T))
    
def vapor_pressure_water(T):
    return ENHANCEMENT_FACTOR * np.exp(K1w/T + K2w + K3w*T + K4w*T*T + K5w*np.log(T))

j = 0
for i in range(4):
    for j in range(3):
        elem=4*j+i
        e = vapor_pressure_water(tK[elem])
        p = rh[elem] * e
        AH = p * 100 * 1000 / (461.5 * tK[elem]) # 100 for hPa->Pa, 1000 for kg->g, 461.5 for specific gas constant
        print("& {:10.2f}".format(AH), "&", "{:10.2f}".format(p), " \\\\" )