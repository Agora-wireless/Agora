import sys
import re
import pandas as pd
import numpy as np
import statistics as sts
from matplotlib import pyplot as plt
import dataconf as dc

def series_str2float(ser):
  for idx, val in ser.items():
    if isinstance(val, str):
      ser[idx] = float(re.sub(r"([A-Z]+)|([a-z]+)", '', val))
  return ser

if len(sys.argv) != 2:
  print('usage: python3 ' + sys.argv[0] + ' <number of devices>')
  exit()

num_dev = int(sys.argv[1])
num_file = len(dc.files)
xlabel = dc.cols[0][0]
ylabels = [dc.cols[0][-1], dc.cols[1][-1], dc.cols[2][-1]]

dfs = []
for i in range(num_file):
  for j in range(num_dev):
    df = pd.read_csv(dc.files[i] + f'-{j}.csv', usecols=dc.cols[i])
    dfs.append(df)
    xminmax = [min(df[xlabel]), max(df[xlabel])]
    if dc.xlims[j] == []:
      dc.xlims[j] = xminmax
    else:
      if dc.xlims[j][0] < xminmax[0]:
        dc.xlims[j][0] = xminmax[0]
      if dc.xlims[j][1] > xminmax[1]:
        dc.xlims[j][1] = xminmax[1]

if dc.enable_trace_plot:
  fig, axs = plt.subplots(num_file, 1, constrained_layout=True)
  fig.suptitle(dc.title + ' Traces')
  for i in range(num_file):
    for j in range(num_dev):
      idx = i * num_dev + j
      axs[i].plot(dfs[idx][xlabel], series_str2float(dfs[idx][ylabels[i]].copy()))
    axs[i].set_xlim(dc.xlims[0])
    if dc.ylims[i] != []:
      axs[i].set_ylim(dc.ylims[i])
    axs[i].set_xlabel(xlabel)
    axs[i].set_ylabel(ylabels[i])

if dc.enable_stats_plot:
  fig, axs = plt.subplots(num_file, 1, constrained_layout=True)
  fig.suptitle(dc.title + ' Statistics')
for i in range(num_file):
  for j in range(num_dev):
    idx = i * num_dev + j
    data_select = dfs[idx][ylabels[i]][dfs[idx][xlabel] \
                  .between(dc.xlims[j][0], dc.xlims[j][1])]
    series_str2float(data_select)
    if dc.stats_zero_fill[i]:
      len_zeros = (dc.xlims[j][1] - dc.xlims[j][0] + 1) - len(data_select)
      if len_zeros > 0:
        data_select = pd.concat([data_select, pd.Series(np.zeros(len_zeros))])
    if dc.enable_stats_plot:
      axs[i].plot(np.sort(data_select), \
          1. * np.arange(len(data_select)) / (len(data_select) - 1))
    print(ylabels[i] + f': Mean[{j}] = %f, ' % sts.mean(data_select) + \
          f'Median[{j}] = %f' % sts.median(data_select))
    if ylabels[i] == 'Symbol-Errors':
      print(f'SER[{j}] = %f' % (sts.mean(data_select) / dc.qam_symbols_per_frame))
  if dc.enable_stats_plot:
    axs[i].set_xlabel(ylabels[i])
    axs[i].set_ylabel('CDF')
if dc.enable_trace_plot or dc.enable_stats_plot:
  plt.show()
