% DO NOT RUN it TOGETHER WITH INSPECT_SINGLE_TEST!

clc
clear

dataset_filename = "UeRxData-loc1.h5"; % changeable
inspect_frame = 100; % changeable
verbose = "true"; % changeable 
group_id = '/Data'; % fixed
% grab params from .h5 file
samples_per_slot = double(h5readatt(dataset_filename, group_id, 'SLOT_SAMP_LEN'));
tx_zero_prefix_len = double(h5readatt(dataset_filename, group_id, 'TX_ZERO_PREFIX_LEN'));
data_size = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_NUM'));
data_start = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_START'));
data_stop = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_STOP'));
fft_size = double(h5readatt(dataset_filename, group_id, 'OFDM_CA_NUM'));
cp_len = double(h5readatt(dataset_filename, group_id, 'CP_LEN'));
total_dl_symbols = double(h5readatt(dataset_filename, group_id, 'DL_SLOTS'));
dl_pilot_symbols = double(h5readatt(dataset_filename, group_id, 'DL_PILOT_SLOTS'));
total_users = 1;
beacon_syms = 1;

dl_data_symbols = total_dl_symbols - dl_pilot_symbols;


%% PROCESSING RXDATA (Channel Estimation and Equalization)
start_id = cp_len + tx_zero_prefix_len;
snr = zeros(1, total_users);
evm = zeros(1, total_users);
nz_start_idx = (fft_size - data_size)/2;
nz_sc_idx = nz_start_idx+1:nz_start_idx+data_size;
plt_trk_sp = 16;
data_sc_idx = setdiff(1:data_size, 1:plt_trk_sp:data_size);
data_phase_corr = zeros(data_size, dl_data_symbols, total_users);

for u=1:total_users
    % Process pilots
    ch_est = zeros(data_size, dl_pilot_symbols);

    for p=1:dl_pilot_symbols
        rx_pilot_f_tmp = fftshift(fft(rx_pilot_cxdouble(start_id + 1:start_id + fft_size, u, p)));
        ch_est(:, p) = rx_pilot_f_tmp(nz_sc_idx) ./ tx_pilot_cxdouble(:, u, p);
    end
    if dl_pilot_symbols == 1
        ch_est_mean = ch_est;
    else
        ch_est_mean = mean(ch_est, 2);
    end
    %Process data symbols
    aevms = zeros(u, dl_data_symbols);
    



end