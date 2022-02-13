import sys
import re
import pandas as pd
import numpy as np
import statistics as sts
from matplotlib import pyplot as plt
import dataconf as dc

def array2float(arr):
	for i in range(arr.index[0], arr.index[-1]):
		if isinstance(arr[i], str):
			arr[i] = float(re.sub(r"([A-Z]+)|([a-z]+)", '', arr[i]))
	return arr

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
		dfs.append(pd.read_csv(dc.files[i] + '-' + str(j) + '.csv', usecols=dc.cols[i]))

if dc.enable_trace_plot:
	fig, axs = plt.subplots(num_file, 1, constrained_layout=True)
	fig.suptitle(dc.title + ' Traces')
	for i in range(num_file):
		for j in range(num_dev):
			idx = i * num_dev + j
			axs[i].plot(dfs[idx][xlabel], array2float(dfs[idx][ylabels[i]].copy()))
		axs[i].set_xlim(dc.xlims[0])
		if bool(dc.ylims[i]):
			axs[i].set_ylim(dc.ylims[i])
		axs[i].set_xlabel(xlabel)
		axs[i].set_ylabel(ylabels[i])
	plt.show()

if dc.enable_stats_plot:
	fig, axs = plt.subplots(num_file, 1, constrained_layout=True)
	fig.suptitle(dc.title + ' Statistics')
for i in range(num_file):
	for j in range(num_dev):
		idx = i * num_dev + j
		data_select = dfs[idx][ylabels[i]] \
				[(dc.xlims[j][0] <= dfs[idx][xlabel]) & (dfs[idx][xlabel] < dc.xlims[j][1])]
		array2float(data_select)
		if dc.stats_zero_fill[i]:
			len_zeros = (dc.xlims[j][1] - dc.xlims[j][0]) - len(data_select)
			data_select = pd.concat([data_select, pd.Series(np.zeros(len_zeros))])
		print(data_select)
		if dc.enable_stats_plot:
			axs[i].plot(np.sort(data_select), \
					1. * np.arange(len(data_select)) / (len(data_select) - 1))
		print(ylabels[i] + ': Mean[' + str(j) + '] = % f' % sts.mean(data_select) \
				+ ', Median[' + str(j) + '] = %f' % sts.median(data_select))
		if ylabels[i] == 'Symbol-Errors':
			print('SER[' + str(j) + '] = %f' % \
					(sts.mean(data_select) / dc.qam_symbols_per_frame))
	if dc.enable_stats_plot:
		axs[i].set_xlabel(ylabels[i])
		axs[i].set_ylabel('CDF')
if dc.enable_stats_plot:
	plt.show()
