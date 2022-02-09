title = 'Downlink User and Listener'
cols = [['Frame', 'EVM-SNR'], ['Frame', 'RF-SNR'], ['Frame', 'Symbol-Errors']]
files = ['uedata-evmsnr', 'uedata-rfsnr', 'uedata-se']
xlims = [[0,10000] for i in range(20)]
ylims = [[-30,30], [0,20], []]
enable_trace_plot = True
enable_stats_plot = True
qam_symbols_per_frame = 304
