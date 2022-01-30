import sys
import pandas as pd
from matplotlib import pyplot as plt
#if len(sys.argv) != 2:
#  print('usage: python3', str(sys.argv[0]), '<data_file>')
#  exit()
columns_evmsnr = ["Frame", "User", "SNR"]
columns_se = ["Frame", "User", "SymbolErrors"]
columns_rfsnr = ["Frame", "AntId", "RFSNR"]
df_evmsnr = pd.read_csv("uedata-evmsnr.csv", usecols=columns_evmsnr)
df_se = pd.read_csv("uedata-se.csv", usecols=columns_se)
df_rfsnr = pd.read_csv("uedata-rfsnr.csv", usecols=columns_rfsnr)
num_users = 1
range_frame = [0,698000]
for i in range(num_users):
	fig, axs = plt.subplots(3, 1, constrained_layout=True)
	fig.suptitle('Single User Downlink')
	df_select = df_evmsnr[df_evmsnr.User == i]
	axs[0].plot(df_select.Frame, df_select.SNR)
	axs[0].set_xlim(range_frame)
	axs[0].set_ylim([-30,30])
	axs[0].set_xlabel('Frame')
	axs[0].set_ylabel('EVM-SNR (dB)')
	
	df_select = df_rfsnr[df_rfsnr.AntId == i]
	axs[1].plot(df_select.Frame, df_select.RFSNR)
	axs[1].set_xlim(range_frame)
	axs[1].set_ylim([0,20])
	axs[1].set_xlabel('Frame')
	axs[1].set_ylabel('RF-SNR (dB)')
	
	df_select = df_se[df_se.User == i]
	axs[2].plot(df_select.Frame, df_select.SymbolErrors)
	axs[2].set_xlim(range_frame)
	axs[2].set_ylim([0,40])
	axs[2].set_xlabel('Frame')
	axs[2].set_ylabel('Number of Symbol Errors')
	plt.show()
