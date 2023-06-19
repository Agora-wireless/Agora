import time
import matplotlib.pyplot as plt
import numpy as np
import os
import struct

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

def plot_coordinates(coordinates, angle=np.pi/10, ax=None):
    i = np.array([coord[0] for coord in coordinates])
    q = np.array([coord[1] for coord in coordinates])
    ax.scatter(i * np.cos(angle) + q * np.sin(angle), - i * np.sin(angle) + q * np.cos(angle), s=10)
    ax.set_xlabel('I', fontsize=18)
    ax.set_ylabel('Q', fontsize=18)
    ax.set_xticks(np.arange(-1.5, 1.5, 0.5))
    ax.set_yticks(np.arange(-1.5, 1.5, 0.5))
    ax.tick_params(axis='both', which='major', labelsize=12)
    ax.set_title('Signal Constellation', fontsize=22)

def update_plot(file_path):
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

    while True:
        if os.path.isfile("equal_data.bin"):
            # Call the update_plot function
            update_plot("equal_data.bin")
            break
        else:
            # print("File 'equal_data.bin' not found. Waiting...")
            time.sleep(0.01)  # Wait for 1 second before checking again

if __name__ == "__main__":
    main()
