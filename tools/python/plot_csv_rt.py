import sys
import numpy as np
import matplotlib.pyplot as plt

xaxis_max = 1000
yaxis_min = 0
yaxis_max = 30
refresh_rate = 100
if len(sys.argv) > 1:
  xaxis_max = int(sys.argv[1])
if len(sys.argv) > 2:
  yaxis_max = int(sys.argv[2])
if len(sys.argv) > 3:
  refresh_rate = int(sys.argv[3])

i_row = 0
for line in sys.stdin:
  if i_row == 0:
    plt.ion()
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(211)
    ax2 = fig1.add_subplot(212)
    num_cols = len(line.split(','))
    arr = np.full((xaxis_max, num_cols), np.nan)
    arr[:,0] = np.linspace(0, xaxis_max - 1, xaxis_max)
    arr[0,1] = yaxis_min
    arr[-1,1] = yaxis_max
    curves = ax1.plot(arr[:,0], arr[:,1:])
    ax2.set_ylabel("CDF")
    if len(sys.argv) > 5:
      ax1.set_xlabel(sys.argv[4])
      ax1.set_ylabel(sys.argv[5])
      ax2.set_xlabel(sys.argv[5])
    leg_name = ''
    if len(sys.argv) > 6:
      leg_name = sys.argv[6]
    leg = []
    for j_col in range(1, num_cols):
      leg.append(leg_name + ' ' + str(j_col))
    ax1.legend(leg)
  j_col = 0
  for elem in line.split(','):
    if j_col < num_cols:
      arr[i_row, j_col] = float(elem)
      j_col += 1
  i_row += 1
  if i_row % refresh_rate == 0:
    for j_col in range(1, num_cols):
      curves[j_col - 1].set_xdata(arr[:,0])
      curves[j_col - 1].set_ydata(arr[:,j_col])
    fig1.canvas.draw()
    fig1.canvas.flush_events()
  if arr[i_row - 1, 0] >= xaxis_max - 1:
    break
num_rows = i_row
plt.ioff()

cdf_ydata = 1. * np.arange(num_rows) / (num_rows - 1)
for j_col in range(1, num_cols):
  cdf_xdata = np.sort(arr[:num_rows, j_col])
  ax2.plot(cdf_xdata, cdf_ydata)
ax2.legend(leg)
fig1.canvas.draw()
plt.show()
