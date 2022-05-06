import sys
import re
import pandas as pd
import numpy as np
import statistics as sts
from matplotlib import pyplot as plt

if len(sys.argv) == 3:
  use_csv_logger = sys.argv[2] == 'True'
elif len(sys.argv) == 2:
  use_csv_logger = True
else:
  print('usage: python3 ' + sys.argv[0] + ' <number of UEs>')
  exit()
num_dev = int(sys.argv[1])

# start of customizable plot config #

title = 'Downlink Performance'

enable_trace_plot = True
enable_stats_plot = True
xlims = [[] for i in range(num_dev)]

if use_csv_logger:
  cols = [
    ['Frame', 'UE0-DLP0-SNR'],
    ['Frame', 'UE1-DLP0-SNR'],
    ['Frame', 'UE0-EVM-SNR'],
    ['Frame', 'UE1-EVM-SNR'],
    ['Frame', 'UE0-BER'],
    ['Frame', 'UE1-BER']
  ]
  files = [
    'log-dlpsnr-ue-119.csv',
    'log-evmsnr-ue-119.csv',
    'log-berser-ue-119.csv'
  ]
  stats_zero_fill = [False for i in range(len(cols))]
  ylims = [[] for i in range(len(cols))]

else: # use log txt
  cols = [
    ['Frame', 'DL-Pilot-SNR-0'],
    ['Frame', 'DL-Pilot-Offset-0'],
    ['Frame', 'EVM-SNR'],
    ['Frame', 'Symbol-Errors']
  ]
  files = [
    'uedata-dlpsnr',
    'uedata-dlpoff',
    'uedata-evmsnr',
    'uedata-se'
  ]
  stats_zero_fill = [False, False, False, True]
  ylims = [[], [], [], []]
  qam_symbols_per_frame = 304 - (304 / 16)

# end of customizable plot config #

num_file = len(files)
xlabel = cols[0][0]
ylabels = [cols[i][1] for i in range(len(cols))]

dfs = []
for i in range(num_file):
  for j in range(num_dev):
    df = pd.read_csv(files[i], usecols=cols[i * num_dev + j]).dropna()
    dfs.append(df)
    xminmax = [min(df[xlabel]), max(df[xlabel])]
    if xlims[j] == []:
      xlims[j] = xminmax
    else:
      if xlims[j][0] < xminmax[0]:
        xlims[j][0] = xminmax[0]
      if xlims[j][1] > xminmax[1]:
        xlims[j][1] = xminmax[1]

def str2float(ser):
  for idx, val in ser.items():
    if isinstance(val, str):
      ser[idx] = float(re.sub(r"([A-Z]+)|([a-z]+)", '', val))
  return ser

if enable_trace_plot:
  fig, axs = plt.subplots(num_file, 1, constrained_layout=True)
  fig.suptitle(title)
  for i in range(num_file):
    for j in range(num_dev):
      idx = i * num_dev + j
      axs[i].plot(dfs[idx][xlabel], str2float(dfs[idx][ylabels[idx]].copy()),
                  label = 'UE ' + str(j))
    axs[i].set_xlim(xlims[0])
    if ylims[idx] != []:
      axs[i].set_ylim(ylims[idx])
    axs[i].set_xlabel(xlabel)
    axs[i].set_ylabel(ylabels[idx])
    axs[i].legend()

if enable_stats_plot:
  fig, axs = plt.subplots(num_file, 1, constrained_layout=True)
  fig.suptitle(title)
for i in range(num_file):
  for j in range(num_dev):
    idx = i * num_dev + j
    data_select = dfs[idx][ylabels[idx]][dfs[idx][xlabel] \
                  .between(xlims[j][0], xlims[j][1])]
    str2float(data_select)
    if stats_zero_fill[idx]:
      len_zeros = (xlims[j][1] - xlims[j][0] + 1) - len(data_select)
      if len_zeros > 0:
        data_select = pd.concat([data_select, pd.Series(np.zeros(len_zeros))])
    if enable_stats_plot:
      axs[i].plot(np.sort(data_select),
                  1. * np.arange(len(data_select)) / (len(data_select) - 1),
                  label = 'UE ' + str(j))
    print(ylabels[idx] + f': Mean[{j}] = %f, ' % sts.mean(data_select) +
          f'Median[{j}] = %f' % sts.median(data_select))
    if ylabels[idx] == 'Symbol-Errors':
      print(f'SER[{j}] = %f' % (sts.mean(data_select) / qam_symbols_per_frame))
  if enable_stats_plot:
    axs[i].set_xlabel(ylabels[idx])
    axs[i].set_ylabel('CDF')
    axs[i].legend()
if enable_trace_plot or enable_stats_plot:
  plt.show()