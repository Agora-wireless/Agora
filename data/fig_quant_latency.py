import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
plt.style.use('ggplot')

x = ['Median', '90th percentile', '99th percentile', '99.9th percentile', '99.99th percentile', '99.999th percentile', 'Max']

x_pos = [i for i, _ in enumerate(x)]
y_val = []

input_file = open('frame_latencies.txt', 'r')
lines = input_file.readlines()
data = []
for line in lines:
    data.append(float(line))
data.sort()

size_data = len(data)

idx_mediam = size_data / 2
y_val.append(data[idx_mediam])
idx_90 = int(size_data * 0.9)
y_val.append(data[idx_90])
idx_99 = int(size_data * 0.99)
y_val.append(data[idx_99])
idx_999 = int(size_data * 0.999)
y_val.append(data[idx_999])
idx_9999 = int(size_data * 0.9999)
y_val.append(data[idx_9999])
idx_99999 = int(size_data * 0.99999)
y_val.append(data[idx_99999])
y_val.append(data[-1])

plt.bar(x_pos, y_val, color='green')
plt.xlabel("Percentile")
plt.ylabel("Latency (ms)")

plt.xticks(x_pos, x)

for i, v in enumerate(y_val):
    plt.text(v + 3, i + .25, str(v), 
        color = 'blue', fontweight = 'bold')

plt.savefig("fig_quant_latency.pdf")