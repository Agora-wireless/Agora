function [evm_considx] = inspect_single_frame_showEVM(dataset_filename, inspect_frame, verbose, cons_idx, order)
%%Load data from the input file and inspect evm and snr
%dataset_filename = "UeRxData.h5";
%inspect_frame = 100;
%verbose = "false";
% -------- Fixed Values --------
group_id = '/Data';
%Display file / group attributes and datasets
if verbose == "true"
    h5disp(dataset_filename, group_id);
end

% -- cons_idx is an array that contains indices of transmit constellations of interests, to compare their EVM percentage
% based on all collected frames.
% E.g., for 16-QAM, the value each element of cons_idx can take is from 0
% to 15.
% -- order: 2, 4, 16, 64, which stands for BPSK, QPSK, 16-QAM, and 64-QAM

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

configs = [samples_per_slot tx_zero_prefix_len data_size data_start data_stop fft_size cp_len ...
    total_dl_symbols dl_pilot_symbols total_users];

%Choose the downlink data
dataset_id = '/DownlinkData';

% Dimensions  [Samples, Ant, Symbol, Cells, Frame]
start = [1 1 1 1 inspect_frame];
count = [(samples_per_slot * 2) total_users total_dl_symbols 1 1];      % only do processing on data 'D's
%Display Info
if verbose == "true"
    h5disp(dataset_filename,strcat(group_id,dataset_id));  % show info of downlink dataset
end

%Generate a int16 array
rx_syms_hdf5 = h5read(dataset_filename, strcat(group_id,dataset_id), start, count);
%Convert to double and scale
rx_syms_scaled_double = double(rx_syms_hdf5) ./ double(intmax('int16'));      % error-prone?
clear rx_syms_hdf5;
% Convert to complex double
% Samples x User x Symbol
rx_syms_cxdouble = complex(rx_syms_scaled_double(1:2:end,:,:), rx_syms_scaled_double(2:2:end,:, :));
clear rx_syms_scaled_double;
% Split off pilots and data
rx_pilot_cxdouble = rx_syms_cxdouble(:,:,1:dl_pilot_symbols);
rx_data_cxdouble = rx_syms_cxdouble(:,:,1+dl_pilot_symbols:end);
clear start count;

%Choose the Beacon data
dataset_id = '/BeaconData';

% Dimensions  [Samples, Ant, Symbol, Cells, Frame]
start = [1 1 1 1 inspect_frame];
count = [(samples_per_slot * 2) total_users beacon_syms 1 1];
%Display Info
if verbose == "true"
    h5disp(dataset_filename,strcat(group_id,dataset_id));
end
%Generate a int16 array
rx_beacon_hdf5 = h5read(dataset_filename, strcat(group_id,dataset_id), start, count);
%Convert to double and scale
rx_beacon_scaled_double = double(rx_beacon_hdf5) ./ double(intmax('int16'));
clear rx_beacon_hdf5;
%Convert to complex double
% Samples x User x Symbol
rx_beacon_cxdouble = complex(rx_beacon_scaled_double(1:2:end,:,:), rx_beacon_scaled_double(2:2:end,:, :));
rx_beacon_rssi = process_beacon(rx_beacon_cxdouble, tx_zero_prefix_len);
%     beacon_samp_start = tx_zero_prefix_len + 240; % 15 reps of STS (16-samps) at the start;
%     beacon_samp_len = 256; % two reps of 128-samps Gold code;
%     rx_beacon_rssi = zeros(1, total_users);
%     for u = 1:total_users
%         rx_beacon = rx_beacon_cxdouble(beacon_samp_start+1:beacon_samp_start+beacon_samp_len, u, 1);
%         rx_beacon_rssi(u) = 10*log10((rx_beacon'*rx_beacon) / beacon_samp_len);
%     end
clear rx_beacon_scaled_double;
clear start count;

dataset_id = '/TxPilot';
%*2 for complex type (native float)
total_samples = data_size * 2;
if verbose == "true"
    h5disp(dataset_filename,strcat(group_id,dataset_id));
end
start = [1 1 1 1 1];
count = [total_samples total_users 1 1 1];
tx_pilot_hdf5 = double(h5read(dataset_filename, strcat(group_id,dataset_id), start, count));
%Convert to complex
tx_pilot_cxdouble = complex(tx_pilot_hdf5(1:2:end,:), tx_pilot_hdf5(2:2:end,:));
clear tx_pilot_hdf5 dataset_id start count;

dataset_id = '/TxData';
%Compare the pilot data,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
if verbose == "true"
    h5disp(dataset_filename,strcat(group_id,dataset_id));
end
start = [1 1 1 1 1];
count = [total_samples total_users total_dl_symbols 1 1];
tx_data_hdf5 = double(h5read(dataset_filename, strcat(group_id,dataset_id), start, count));
%Convert to complex type
tx_syms_cxdouble = complex(tx_data_hdf5(1:2:end,:,:), tx_data_hdf5(2:2:end,:,:));
% Samples (complex) x User Ant x Downlink Symbol Id
% removing the pilot for now because it doesn't check out?
tx_pilot_bad = tx_syms_cxdouble(:,:,1);
tx_data_cxdouble = tx_syms_cxdouble(:,:,2:end);
clear start count total_samples tx_data_hdf5 dataset_id;
% disp(isequal(tx_pilot_bad, tx_pilot_cxdouble));
clear tx_pilot_bad;

clear dataset_filename group_id;
clear beacon_syms cp_len data_size data_start data_stop dl_pilot_symbols...
    samples_per_slot tx_zero_prefix_len total_dl_symbols fft_size;

% demul_data is assigned from data_phase_corr from process_rx file
%------
%     [demul_data, data_sc_idx, evm, snr] = ...
%         process_rx_frame(configs, tx_pilot_cxdouble, tx_data_cxdouble, rx_pilot_cxdouble, rx_data_cxdouble);

[demul_data, data_sc_idx, evm, snr, evm_considx] = ...
    process_rx_frame_showEVM(configs, tx_pilot_cxdouble, tx_data_cxdouble, rx_pilot_cxdouble, rx_data_cxdouble, cons_idx, order);

clear configs tx_pilot_cxdouble rx_pilot_cxdouble rx_data_cxdouble;

%Plot Rx waveform
% combined_rx = vertcat(rx_beacon_cxdouble, reshape(squeeze(rx_syms_cxdouble(:,1,:)), [], 1));
% figure('Name', ['Frame ' num2str(inspect_frame) ' Receive WaveForm']);
% tiledlayout(2,1)
% %Top (Real)
% nexttile;
% plot(real(combined_rx));
% axis([0 inf -1 1]);
% title('Rx Real (I)');
% %Bottom (Q)
% nexttile;
% plot(imag(combined_rx));
% axis([0 inf -1 1]);
% title('Rx Imag (Q)');

% 
% for u=1:total_users
%     rx_cnstl = demul_data(data_sc_idx, : , u);
%     tx_cnstl = tx_data_cxdouble(data_sc_idx, :, u);
%     figure('Name', ['Constellation [User ', num2str(u), ']']);
%     pt_size = 18;
%     scatter(real(rx_cnstl(:)), imag(rx_cnstl(:)),pt_size,'r','filled');  % reshape the rx_constl mat into single col
%     hold on
%     pt_size = 250;
%     scatter(real(tx_cnstl(:)), imag(tx_cnstl(:)),pt_size, 'b', 'p', 'filled');
%     title(['Constellation [User ', num2str(u), ', from Frame ', num2str(inspect_frame), ']']);
%     hold on
%     scatter(real(rx_cnstl(:,1)), imag(rx_cnstl(:,1)),7,'green','filled');
%     scatter(real(rx_cnstl(:,2)), imag(rx_cnstl(:,2)),7,'black','filled');
%     legend('Superimposed','Sent','From Slot1','From Slot2')
%     grid on
%     xlabel('I'); ylabel('Q')
% end

clear total_users tx_data_cxdouble rx_beacon_cxdouble rx_syms_cxdouble tx_syms_cxdouble combined_rx;

%disp(['Frame Inspect: ', num2str(inspect_frame)]);
%disp(['Beacon RSSI (dB): ', num2str(rx_beacon_rssi)]);
%disp(['SNR: ', num2str(snr)]);
%disp(['EVM: ', num2str(evm)]);
% disp(['constellations number: ', num2str(length(rx_cnstl(:)))]);

end
