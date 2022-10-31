import sys
import numpy as np
import matplotlib.pyplot as plt

max_rows = 0
if len(sys.argv) > 1:
  max_rows = int(sys.argv[1])

rows = []
for line in sys.stdin:
  row = []
  for elem in line.split(','):
    row.append(float(elem))
  rows.append(row)
  if len(rows) == max_rows:
    break
arr = np.array(rows)
n_rows, n_cols = arr.shape

plt.subplot(2, 1, 1)
plt.plot(arr[:,0], arr[:,1:])
if len(sys.argv) > 3:
  plt.xlabel(sys.argv[2])
  plt.ylabel(sys.argv[3])
if len(sys.argv) > 4:
  leg = []
  for i in range(n_cols):
    leg.append(sys.argv[4] + ' ' + str(i))
  plt.legend(leg)

plt.subplot(2, 1, 2)
plt.ylabel("CDF")
cdf_ydata = 1. * np.arange(n_rows) / (n_rows - 1)
for i in range(1, n_cols):
  cdf_xdata = np.sort(arr[:,i])
  plt.plot(cdf_xdata, cdf_ydata)
if len(sys.argv) > 3:
  plt.xlabel(sys.argv[3])
if len(sys.argv) > 4:
  plt.legend(leg)
plt.show()