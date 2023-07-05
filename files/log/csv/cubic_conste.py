import time
import matplotlib.pyplot as plt
import numpy as np
import os
import struct
from scipy.io import savemat
import argparse

# Constants
#EVM_MIN = +np.inf

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

def plot_coordinates(coordinates, modulation_type, args, angle=np.pi*2, ax=None):
    i = np.array([coord[0] for coord in coordinates])
    q = np.array([coord[1] for coord in coordinates])

    i_1 = i[:int(len(i)/2)-1]
    q_1 = q[:int(len(q)/2)-1]

    i_2 = i[int(len(i)/2):]
    q_2 = q[int(len(q)/2):]

    if args.SISO:
        ax.scatter(i,q,s=10)
    elif args.MIMO:
        ax.scatter(i_1, q_1, s=10)
        ax.scatter(i_2, q_2, s=10, color="red")
    else:
        # Default scatter plot
        ax.scatter(i,q,s=10)
    
    reference_symbols = {
        '16QAM': {
            'I': np.array([-3, -1, 1, 3]) / np.sqrt(10),
            'Q': np.array([-3, -1, 1, 3]) / np.sqrt(10)
        },
        '64QAM': {
            'I': np.array([-7, -5, -3, -1, 1, 3, 5, 7]) / np.sqrt(42),
            'Q': np.array([-7, -5, -3, -1, 1, 3, 5, 7]) / np.sqrt(42)
        }
    }
    reference_i = reference_symbols[modulation_type]['I']
    reference_q = reference_symbols[modulation_type]['Q']

    for ref_i in reference_i:
        for ref_q in reference_q:
            ax.scatter(ref_i, ref_q, s=80, marker='+', color="black")
    
    ax.set_xlabel('I', fontsize=18)
    ax.set_ylabel('Q', fontsize=18)
    ax.set_ylim([-1.5, 1.5])
    ax.set_xlim([-1.5, 1.5])
    ax.set_xticks(np.arange(-1.5, 1.7, 0.5))
    ax.set_yticks(np.arange(-1.5, 1.7, 0.5))
    ax.tick_params(axis='both', which='major', labelsize=12)

def update_plot(file_path, modulation_type, args, EVM_MIN=+np.inf):
    plt.ion()
    fig, ax = plt.subplots()

    while True:
        binary_data = read_binary_file(file_path)
        float_values = convert_to_float(binary_data)
        coordinates = parse_coordinates(float_values)

        ax.clear()
        plot_coordinates(coordinates, modulation_type, args, ax=ax)
        plt.pause(0.05)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--SISO', action='store_true', help='Use SISO scatter plot')
    parser.add_argument('--MIMO', action='store_true', help='Use MIMO scatter plot')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--QAM16', action='store_true', help='Use 16QAM modulation')
    group.add_argument('--QAM64', action='store_true', help='Use 64QAM modulation')
    args = parser.parse_args()

    if args.SISO and args.MIMO:
        print("Please choose only one of --SISO and --MIMO.")
        return

    if not args.SISO and not args.MIMO:
        print("Please choose either --SISO or --MIMO.")
        return

    modulation_type = '16QAM'  # Default modulation type

    if args.SISO:
        print("Using SISO scatter plot.")
    elif args.MIMO:
        print("Using MIMO scatter plot.")

    if args.QAM16:
        print("Using 16QAM modulation.")
        modulation_type = '16QAM'
    elif args.QAM64:
        print("Using 64QAM modulation.")
        modulation_type = '64QAM'
    
    if os.path.isfile("equal_data.bin"):
        os.system("rm equal_data.bin")

    with open('equal_data.bin', 'wb'):
        pass

    while True:
        if os.path.isfile("equal_data.bin"):
            update_plot("equal_data.bin", modulation_type, args)
            break
        else:
            time.sleep(0.1)

if __name__ == "__main__":
    main()

