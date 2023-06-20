import numpy as np
import matplotlib.pyplot as plt
import os
import time


# Function to read data from the file
def read_data():
    # Load the data from the saved file
    data = np.fromfile('data.txt', dtype=np.complex64)

    # Extract real and imaginary parts of the complex numbers
    real_part = data.real
    imaginary_part = data.imag

    return real_part, imaginary_part


# Create the figure with specified figsize
fig = plt.figure(figsize=(10, 8))


# Function to update and redraw the plot
def update_plot():
    # Clear the current plot
    plt.clf()

    # Read the data from the file
    real_part, imaginary_part = read_data()

    # Create a line plot for the real part
    plt.subplot(2, 1, 1)
    plt.plot(real_part, label='Real Part')

    # Create a line plot for the imaginary part
    plt.subplot(2, 1, 1)
    plt.plot(imaginary_part, label='Imaginary Part')

    # Perform FFT on the data
    fft_data = np.fft.fft(real_part + 1j * imaginary_part, n=1024)

    # Create a line plot for the FFT magnitude
    plt.subplot(2, 1, 2)
    plt.plot(np.abs(fft_data), label='FFT Magnitude')

    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('Real, Imaginary, and FFT Magnitude')
    plt.legend()
    plt.pause(0.01)


# Delete the existing data file if it exists
if os.path.exists('data.txt'):
    os.remove('data.txt')

# Create an empty data file
with open('data.txt', 'wb'):
    pass

# Continuously update the plot
while True:
    update_plot()
    time.sleep(0.01)
