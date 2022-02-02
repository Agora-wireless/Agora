import sys
import pandas as pd
import numpy as np
import statistics as sts
from matplotlib import pyplot as plt

columns = [['Frame', 'User', 'EVM-SNR'],
	['Frame', 'User', 'RF-SNR'],
	['Frame', 'User', 'Symbol_Errors']]
filenames = ['uedata-evmsnr.csv', 'uedata-rfsnr.csv', 'uedata-se.csv']
figtitle = 'Single User Downlink'
dfs = []
for i in range(len(columns)):
	dfs.append(pd.read_csv(filenames[i], usecols=columns[i]))
users = np.max(dfs[0].User) + 1
xlabel = 'Frame'
ylabels = [columns[0][-1], columns[1][-1], columns[2][-1]]
ylims = [[-30,30], [0,20], [0,80]]
for i in range(users):
	fig, axs = plt.subplots(len(dfs), 1, constrained_layout=True)
	fig.suptitle(figtitle)
	for j in range(len(dfs)):
		df_select = dfs[j][dfs[j].User == i]
		if j == 0:
			xlim = [0, np.max(df_select[xlabel])]
		axs[j].plot(df_select[xlabel], df_select[ylabels[j]])
		axs[j].set_xlim(xlim)
		axs[j].set_ylim(ylims[j])
		axs[j].set_xlabel(xlabel)
		axs[j].set_ylabel(ylabels[j])
	plt.show()
	
	fig, axs = plt.subplots(len(dfs), 1, constrained_layout=True)
	fig.suptitle(figtitle + ' Statistics')
	for j in range(len(dfs)):
		data_select = dfs[j][dfs[j].User == i][ylabels[j]]
		data_select = data_select[~(np.isnan(data_select) | np.isinf(data_select))]
		axs[j].plot(np.sort(data_select), 
			1. * np.arange(len(data_select)) / (len(data_select) - 1))
		axs[j].set_xlabel(ylabels[j])
		axs[j].set_ylabel('CDF')
		print(ylabels[j] + ' Mean = ', sts.mean(data_select))
		print(ylabels[j] + ' Median = ', sts.median(data_select))
	plt.show()
