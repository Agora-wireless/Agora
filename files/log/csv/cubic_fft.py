import numpy as np
import matplotlib.pyplot as plt

# Load the binary file as a complex NumPy array
fft_vec = np.fromfile('fft_data.txt', dtype=np.complex64)

# Extract the real and imaginary parts
real_part = fft_vec.real
imaginary_part = fft_vec.imag

# Plot the real and imaginary parts
plt.figure()


plt.plot(abs(fft_vec))
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('FFT Vector')
plt.legend()
plt.show()
