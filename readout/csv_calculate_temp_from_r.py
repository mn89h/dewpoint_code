import csv

# See https://www.bipm.org/documents/20126/41773843/Guide_ITS-90_5_SPRT_2021.pdf/c4bbbe56-4118-eef7-47cb-3ea234db40b8
# for detailled information

# ITS-90 Resistance values
Rtripel = 100.022
a8 = -2.16E-04
b8 = -8.52E-05

# SPRT reference
D_n = [439.932854, 472.41802, 37.684494, 7.472018, 2.920828, 0.005184, -0.963864, -0.188732, 0.191203, 0.049025]

# Series values
# X = #value from csv
N = 0
M = 1

def series_sum(x, initial_power, step_size, coefficients):
    result = 0
    power = initial_power

    for coeff in coefficients:
        result += coeff * (x ** power)
        power += step_size

    return result

def calc_W(Rmeas):
    return Rmeas/Rtripel

def calc_delta_W(W):
    return a8 * (W - 1) + b8 * (W - 1)**2

def calc_Wr(W, delta_W):
    return W - delta_W

def calc_T(Wr):
    x = (Wr - 2.64) / 1.64
    return 273.15 + series_sum(x, N, M, D_n)

def convert_KelvinToDegree(T_kelvin):
    return T_kelvin - 273.15

def process_csv(input_file, output_file):
    with open(input_file, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)

    processed_data = []
    for row in data[0:2]:
        processed_data.append(row)

    for row in data[2:len(data)]:
        if row:  # Check if the row is not empty
            Rmeas = float(row[0])
            W = calc_W(Rmeas)
            delta_W = calc_delta_W(W)
            Wr = calc_Wr(W, delta_W)
            T_kelvin = calc_T(Wr)
            T_celsius = convert_KelvinToDegree(T_kelvin)
            row[1] = T_celsius
            processed_data.append(row)

    with open(output_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(processed_data)

# Replace 'input.csv' and 'output.csv' with your file paths
filename = "measurements_20231222-150942"
process_csv(filename + '.csv', filename + '_processed.csv')
