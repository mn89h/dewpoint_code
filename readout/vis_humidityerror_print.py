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

print(H_real*100)
print(H_off*100)
print(np.abs(H_off-H_real)*100)