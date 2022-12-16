import sys
import numpy as np
import matplotlib.pyplot as plt

max_rows = 1000
refresh_rows = 20
yaxis_min = -10
yaxis_max = 30
if len(sys.argv) > 1:
  max_rows = int(sys.argv[1])

i_row = 0
for line in sys.stdin:
  if i_row == 0:
    plt.ion()
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(211)
    ax2 = fig1.add_subplot(212)
    num_cols = len(line.split(','))
    arr = np.full((max_rows, num_cols), np.nan)
    arr[:,0] = np.linspace(0, max_rows - 1, max_rows)
    arr[0,1] = yaxis_min
    arr[-1,1] = yaxis_max
    curves = ax1.plot(arr[:,0], arr[:,1:])
    ax2.set_ylabel("CDF")
    if len(sys.argv) > 3:
      ax1.set_xlabel(sys.argv[2])
      ax1.set_ylabel(sys.argv[3])
      ax2.set_xlabel(sys.argv[3])
    if len(sys.argv) > 4:
      leg = []
      for j_col in range(1, num_cols):
        leg.append(sys.argv[4] + ' ' + str(j_col))
      ax1.legend(leg)
  if i_row < max_rows:
    j_col = 0
    for elem in line.split(','):
      if j_col < num_cols:
        arr[i_row, j_col] = float(elem)
        j_col += 1
    i_row += 1
    if i_row % refresh_rows == 0:
      for j_col in range(1, num_cols):
        curves[j_col - 1].set_xdata(arr[:,0])
        curves[j_col - 1].set_ydata(arr[:,j_col])
      fig1.canvas.draw()
      fig1.canvas.flush_events()
  else:
    break
num_rows = i_row
plt.ioff()

cdf_ydata = 1. * np.arange(num_rows) / (num_rows - 1)
for j_col in range(1, num_cols):
  cdf_xdata = np.sort(arr[:num_rows, j_col])
  ax2.plot(cdf_xdata, cdf_ydata)
if len(sys.argv) > 4:
  ax2.legend(leg)
fig1.canvas.draw()
plt.show()

