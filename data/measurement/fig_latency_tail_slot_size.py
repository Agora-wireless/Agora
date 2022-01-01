import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np

data_file = open('fig_latency_tail_slot_size.txt', 'r')
lines = data_file.readlines()
labels = []
x_values = []
y_values = []

first_line = lines[0]
lines = lines[1:]
strs = first_line.split()
if strs[0] != 'Data':
    print("Invalid data!")
    exit(1)
for i in range(1, len(strs)):
    labels.append(strs[i].strip())
    y_values.append([])

for line in lines:
    values = line.split()
    x_values.append(float(values[0]))
    for i in range(len(labels)):
        y_values[i].append(float(values[i+1]))

fig, ax = plt.subplots(figsize=(4.5, 3.8))  # Create a figure containing a single axes.
for i in range(len(labels)):
    ax.plot(x_values, y_values[i], label=labels[i])  # Plot some data on the axes.
ax.set_xlabel('Slot size', fontsize=10)
ax.set_ylabel('99.9-th Tail Latency (ms)', fontsize=10)
ax.set_xlim([0.42, 2.08])
ax.set_xticks([0.5, 1, 1.5, 2])
ax.set_yticks([0.5, 1, 1.5, 2, 2.5])
ax.grid()
ax.legend(prop={'size': 10})

plt.xticks(fontsize=10)
plt.yticks(fontsize=10)

plt.savefig("fig_latency_tail_slot_size.pdf")