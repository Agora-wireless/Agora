import time
import matplotlib.pyplot as plt
import numpy as np
import os
import struct
from scipy.io import savemat

#EVM_MIN = +np.inf
MODULATION_TYPE = '16QAM'

def read_binary_file(file_path):
    try:
        with open(file_path, 'rb') as file:
            data = file.read()
    except FileNotFoundError:
        print(f"File '{file_path}' not found.")
    except IOError as e:
        print(f"Error reading file: {e}")
    return data

def convert_to_float(binary_data):
    float_values = struct.unpack('f' * (len(binary_data) // 4), binary_data)
    return float_values

def parse_coordinates(data):
    coordinates = []
    for i in range(0, len(data), 2):
        real = data[i]
        imag = data[i + 1]
        coordinates.append((real, imag))
    return coordinates

def plot_coordinates(coordinates, angle=np.pi*2, ax=None):
    i = np.array([coord[0] for coord in coordinates])
    q = np.array([coord[1] for coord in coordinates])
     
    i_1 = i[:int(len(i)/2)-1]
    q_1 = q[:int(len(q)/2)-1]

    i_2 = i[int(len(i)/2):]
    q_2 = q[int(len(q)/2):]
    
    # Calculate EVM
    reference_symbols = {
        '16QAM': {
            'I': np.array([-3, -1, 1, 3]) / np.sqrt(10),
            'Q': np.array([-3, -1, 1, 3]) / np.sqrt(10)
        }
    }
    reference_i = reference_symbols[MODULATION_TYPE]['I']
    reference_q = reference_symbols[MODULATION_TYPE]['Q']
    
    ''' 
    error_i = []
    error_q = []
    for coord in coordinates:
    	error_i.append(np.min(np.abs(coord[0]-reference_i)))
    	error_q.append(np.min(np.abs(coord[1]-reference_q)))
    error_i = np.array(error_i)
    error_q = np.array(error_q)	
    
    evm = np.sqrt(np.mean(error_i**2 + error_q**2))
    evm_percentage = evm * 100
    '''
    '''
    if evm_percentage < EVM_MIN:
        EVM_MIN = evm_percentage
        savemat("result.mat", {
            'evm': evm_percentage,
             'real': i, 'imag': q})
    '''
        # Plot scatter plot
        #ax.scatter(i,q,s=10)
    #ax.clear()
    ax.scatter(i_1, q_1, s=10)
    ax.scatter(i_2, q_2, s=10, color="red")
        #for ref_i in [-3, -1, 1, 3]:
           # for ref_q in [-3, -1, 1, 3]:
                #:ax.scatter(ref_i/np.sqrt(10), ref_q/np.sqrt(10), s=80, marker='+', color = "black")
    ax.set_xlabel('I', fontsize=18)
    ax.set_ylabel('Q', fontsize=18)
    ax.set_ylim([-1.5, 1.5])
    ax.set_xlim([-1.5, 1.5])
    ax.set_xticks(np.arange(-1.5, 1.7, 0.5))
    ax.set_yticks(np.arange(-1.5, 1.7, 0.5))
    ax.tick_params(axis='both', which='major', labelsize=12)
    #ax.set_title('EVM: {evm_percentage:.2f)'%(EVM_MIN), fontsize=22)
    ax.set_title('Signal Constellation', fontsize=22)

    #return EVM_MIN

def update_plot(file_path, EVM_MIN=+np.inf):
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()

    while True:
        binary_data = read_binary_file(file_path)
        float_values = convert_to_float(binary_data)
        coordinates = parse_coordinates(float_values)

        ax.clear()  # Clear the previous plot
        plot_coordinates(coordinates, ax=ax)
        plt.pause(0.05)  # Pause for 0.05 seconds


def main():
    # Remove the file if it exists
    if os.path.isfile("equal_data.bin"):
        os.system("rm equal_data.bin")
    
    with open('equal_data.bin', 'wb'):
        pass
    
    while True:
        if os.path.isfile("equal_data.bin"):
            # Call the update_plot function
            update_plot("equal_data.bin")
            break
        else:
            # print("File 'equal_data.bin' not found. Waiting...")
            time.sleep(0.1)  # Wait for 1 second before checking again

if __name__ == "__main__":
    main()
