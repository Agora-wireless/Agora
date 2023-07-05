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
fig = plt.figure(figsize=(8, 5))

# Create subplots for real and imaginary parts
ax1 = fig.add_subplot(2, 1, 1)

# Create subplot for FFT magnitude
ax2 = fig.add_subplot(2, 1, 2)

# Function to update and redraw the plot
def update_plot():
    # Clear the current plots
    ax1.clear()
    ax2.clear()

    # Read the data from the file
    real_part, imaginary_part = read_data()

    # Create a line plot for the real part
    ax1.plot(real_part, label='Real Part')

    # Create a line plot for the imaginary part
    ax1.plot(imaginary_part, label='Imaginary Part')

    # Perform FFT on the data
    fft_data = np.fft.fft(real_part + 1j * imaginary_part, n=1024)

    # Calculate the FFT frequencies
    fft_freq = np.fft.fftfreq(len(fft_data))

    # Create a line plot for the FFT magnitude
    ax2.plot(fft_freq, 20 * np.log10(np.abs(fft_data)), label='FFT')

    ax1.set_xlabel('Samples', fontsize=18)
    ax1.set_ylabel('Amplitude', fontsize=18 )
    ax1.set_title('Raw IQ Samples', fontsize=20)
    ax1.tick_params(axis='both', which='major', labelsize=12)
    ax1.set_ylim([-0.025, 0.025])

    ax2.set_xlabel('Frequency', fontsize=18)
    ax2.set_ylabel('dB scale', fontsize=18)
    ax2.set_title('FFT', fontsize=20)
    ax2.tick_params(axis='both', which='major', labelsize=12)
    ax2.set_ylim([-70, 0])

    ax1.legend()
    ax2.legend()

    plt.tight_layout()
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

