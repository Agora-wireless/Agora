import numpy as np
import matplotlib.pyplot as plt

# Define the filename
filename = "demod_data_16QAM_test2.bin"

# Read the binary file into a NumPy array
data = np.fromfile(filename, dtype=np.complex64)

# Reshape the data into the appropriate dimensions (if needed)
# For example, if the symbols are arranged as complex_float[row][col], you can reshape as:
# num_rows = ...
# num_cols = ...
# data = data.reshape((num_rows, num_cols))

# Separate the real and imaginary components
real_part = np.real(data)
imaginary_part = np.imag(data)

# Plot the constellation diagram
plt.scatter(real_part, imaginary_part)
plt.xlabel('Real')
plt.ylabel('Imaginary')
plt.title('Constellation Diagram')
plt.grid(True)
plt.show()
