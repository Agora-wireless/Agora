function inspect_agora_results(dataset_filename, verbose)
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

    total_frames = double(h5readatt(dataset_filename, group_id, 'MAX_FRAME'));
    samples_per_slot = double(h5readatt(dataset_filename, group_id, 'SLOT_SAMP_LEN'));
    tx_zero_prefix_len = double(h5readatt(dataset_filename, group_id, 'TX_ZERO_PREFIX_LEN'));
    data_size = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_NUM'));
    data_start = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_START'));
    data_stop = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_STOP'));
    fft_size = double(h5readatt(dataset_filename, group_id, 'OFDM_CA_NUM'));
    cp_len = double(h5readatt(dataset_filename, group_id, 'CP_LEN'));
    total_dl_symbols = double(h5readatt(dataset_filename, group_id, 'DL_SLOTS'));
    dl_pilot_symbols = double(h5readatt(dataset_filename, group_id, 'DL_PILOT_SLOTS'));
    cl_sdr_id = h5readatt(dataset_filename, group_id, 'CL_SDR_ID');

    %Choose the Beacon data
    dataset_id = '/BeaconData';

    %Display Info
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end
    %Generate a int16 array
    rx_beacon_hdf5 = h5read(dataset_filename, strcat(group_id,dataset_id));
    % Dimensions  [Samples, Ant, Symbol, Cells, Frame]
    total_users = size(rx_beacon_hdf5, 2);
    %Convert to double and scale
    rx_beacon_scaled_double = double(rx_beacon_hdf5) ./ double(intmax('int16'));
    clear rx_beacon_hdf5;
    %Convert to complex double
    % Samples x User x Symbol
    rx_beacon_cxdouble = complex(rx_beacon_scaled_double(1:2:end, :, :, 1:total_frames), rx_beacon_scaled_double(2:2:end,:, :, 1:total_frames));
    rx_beacon_rssi = process_beacon(rx_beacon_cxdouble, tx_zero_prefix_len);
    clear rx_beacon_scaled_double;

    experiment = 'MU-MIMO';
    if total_users == 1
        experiment = 'SU-MIMO';
    end

    % New (Beacon RSSI)
    figure('Name', 'Beacon');
    set(gca,'FontSize',18);
    plot(rx_beacon_rssi.', 'LineWidth',2)
    axis([0 total_frames -50 0]);
    ylabel('Beacon RSSI (dB)')
    xlabel('Frame')
    title(['Rx Beacon Power in ' experiment ' Experiment'])
    legend(cl_sdr_id)

    if total_dl_symbols == 0
        return
    end

    %Choose the downlink data
    dataset_id = '/DownlinkData';

    %Display Info
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end
    %Generate a int16 array
    rx_syms_hdf5 = h5read(dataset_filename, strcat(group_id,dataset_id));
    % Dimensions  [Samples, Ant, Symbol, Cells, Frame]
    total_users = size(rx_syms_hdf5, 2);
    %Convert to double and scale
    rx_syms_scaled_double = double(rx_syms_hdf5) ./ double(intmax('int16'));
    clear rx_syms_hdf5;
    %Convert to complex double
    % Samples x User x Symbol x Frame
    rx_syms_cxdouble = complex(rx_syms_scaled_double(1:2:end, :, :, 1:total_frames), rx_syms_scaled_double(2:2:end,:, :, 1:total_frames));
    clear rx_syms_scaled_double;
    % Split off pilots and data
    rx_pilot_cxdouble = rx_syms_cxdouble(:,:,1:dl_pilot_symbols, :);
    rx_data_cxdouble = rx_syms_cxdouble(:,:,1+dl_pilot_symbols:end, :);

    configs = [samples_per_slot tx_zero_prefix_len data_size data_start data_stop fft_size cp_len ...
    total_dl_symbols dl_pilot_symbols total_users];

    dataset_id = '/TxPilot';
    %*2 for complex type (native float)
    total_samples = data_size * 2;
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end 
    start = [1 1 1 1 1];
    count = [total_samples total_users dl_pilot_symbols 1 1];
    tx_pilot_hdf5 = double(h5read(dataset_filename, strcat(group_id,dataset_id), start, count));
    %Convert to complex
    tx_pilot_cxdouble = complex(tx_pilot_hdf5(1:2:end,:,:), tx_pilot_hdf5(2:2:end,:,:));
    clear tx_pilot_hdf5 dataset_id start count;

    dataset_id = '/TxData';
    %Compare the pilot data
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end
    start = [1 1 1 1 1];
    count = [total_samples total_users total_dl_symbols 1 1];
    tx_data_hdf5 = double(h5read(dataset_filename, strcat(group_id,dataset_id), start, count));
    %Convert to complex type
    tx_syms_cxdouble = complex(tx_data_hdf5(1:2:end,:,:), tx_data_hdf5(2:2:end,:,:));
    % Samples (complex) x User Ant x Downlink Data Symbol Id
    tx_data_cxdouble = tx_syms_cxdouble(:, :, dl_pilot_symbols + 1:end);
    clear start count total_samples tx_data_hdf5 dataset_id;
    clear cp_len data_size data_start data_stop dl_pilot_symbols samples_per_slot tx_zero_prefix_len total_dl_symbols fft_size;

    evm = zeros(total_users, total_frames);
    snr = zeros(total_users, total_frames);
    rf_snr = zeros(total_users, total_frames);
    for i=1:total_frames
        [~, ~, evm(:, i), snr(:, i), rf_snr(:, i)] = process_rx_frame(configs, tx_pilot_cxdouble, tx_data_cxdouble, rx_pilot_cxdouble(:, :, :, i), rx_data_cxdouble(:, :, :, i));
    end
    clear configs tx_pilot_cxdouble tx_data_cxdouble rx_pilot_cxdouble rx_data_cxdouble;

    %Plot SNR & EVM Results
    figure('Name', ['File ' dataset_filename]);
    tiledlayout(3,1)
    % Top (SNR)
    nexttile;
    set(gca,'FontSize',18);
    plot(rf_snr.', 'LineWidth',2);
    ylabel('Pilot SNR (dB)')
    title([experiment ' Beamforming SNR & EVM plots'])
    legend(cl_sdr_id)
    nexttile;
    plot(snr.', 'LineWidth',2);
    ylabel('EVM SNR (dB)')
    %Bottom (EVM)
    nexttile;
    plot(evm.', 'LineWidth',2);
    axis([0 total_frames 0 (4 * max(mean(evm, 2)))]);
    ylabel('EVM (%)')

    clear rx_beacon_cxdouble rx_syms_cxdouble tx_syms_cxdouble total_users ;
end
