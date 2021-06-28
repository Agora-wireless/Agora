import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np

data_file = open('cdf_latency.txt', 'r')
lines = data_file.readlines()
x_values = []
y_values = []

for line in lines:
    values = line.split()
    x_values.append(float(values[0]))
    y_values.append(float(values[1]))

fig, ax = plt.subplots()  # Create a figure containing a single axes.
ax.plot(x_values, y_values)  # Plot some data on the axes.
ax.set_xlabel('Latency (ms)')
ax.set_ylabel('CDF')
ax.set_xlim([0, 5])

plt.savefig("fig_cdf_latency.pdf")