import csv
import numpy as np


# read theta-bias table
def read_bias_table(csv_file):
    read = np.loadtxt (csv_file, delimiter = ',' , skiprows=1, dtype="str")

    data = np.zeros((len(read), 7))
    for i in range(len(read)):
        data[i,0] = float(read[i,0])
        
        string = read[i,1]
        float_list = [float(x) for x in string.strip("[]").split()]
        data[i,1:len(float_list)+1] = float_list
        
    #print(data.shape)
    return data


# 内挿
def interpolate_elements(data, target):
    sorted_data = sorted(data, key=lambda x: x[0])  # 一番目の値でソート

    interpolated_values = None
    for i in range(len(sorted_data) - 1):
        if sorted_data[i][0] <= target < sorted_data[i + 1][0]:
            x1, *values1 = sorted_data[i]
            x2, *values2 = sorted_data[i + 1]

            # 内挿
            ratio = (target - x1) / (x2 - x1)
            interpolated_values = [v1 + ratio * (v2 - v1) for v1, v2 in zip(values1, values2)]
            break

    return interpolated_values


# Caliculate bias from theta
def find_bias(data, theta):
    bias = np.zeros(6)
    bias = interpolate_elements(data, theta)
    return bias
    """
    if interpolated_values is not None:
        print(interpolated_values)
    else:
        print("No interpolation performed for the given target.")
    """ 
    
    
    return bias


def main():
    csv_file = "data/csv/save/20230517_183928inwhist_calibration.csv"
    table = read_bias_table(csv_file)
    theta = 12
    bias = find_bias(table, theta)
    print(bias)



if __name__ == '__main__':
    main()