function inspect_single_frame(dataset_filename, inspect_frame, verbose)
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

    samples_per_slot = double(h5readatt(dataset_filename, group_id, 'SLOT_SAMP_LEN'));
    tx_zero_prefix_len = double(h5readatt(dataset_filename, group_id, 'TX_ZERO_PREFIX_LEN'));
    data_size = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_NUM'));
    data_start = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_START'));
    data_stop = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_STOP'));
    fft_size = double(h5readatt(dataset_filename, group_id, 'OFDM_CA_NUM'));
    cp_len = double(h5readatt(dataset_filename, group_id, 'CP_LEN'));
    total_dl_symbols = double(h5readatt(dataset_filename, group_id, 'DL_SLOTS'));
    dl_pilot_symbols = double(h5readatt(dataset_filename, group_id, 'DL_PILOT_SLOTS'));
    total_users = double(h5readatt(dataset_filename, group_id, 'CL_NUM'));
    beacon_syms = 1;

    configs = [samples_per_slot tx_zero_prefix_len data_size data_start data_stop fft_size cp_len ...
    total_dl_symbols dl_pilot_symbols total_users];

    %Choose the downlink data
    dataset_id = '/DownlinkData';

    %Display Info
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id, dataset_id));
    end
    % Dimensions  [Samples, Ant, Symbol, Cells, Frame]
    start = [1 1 1 1 inspect_frame];
    count = [(samples_per_slot * 2) total_users total_dl_symbols 1 1];
    %Generate a int16 array
    rx_syms_hdf5 = h5read(dataset_filename, strcat(group_id, dataset_id), start, count);
    %Convert to double and scale
    rx_syms_scaled_double = double(rx_syms_hdf5) ./ double(intmax('int16'));
    clear rx_syms_hdf5;
    %Convert to complex double
    % Samples x User x Symbol
    rx_syms_cxdouble = complex(rx_syms_scaled_double(1:2:end, :, :), rx_syms_scaled_double(2:2:end, :, :));
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

    clear dataset_filename group_id;
    clear beacon_syms cp_len data_size data_start data_stop dl_pilot_symbols samples_per_slot tx_zero_prefix_len total_dl_symbols fft_size;
    [demul_data, data_sc_idx, evm, snr, rf_snr, tx_waveform] = process_rx_frame(configs, tx_pilot_cxdouble, tx_data_cxdouble, rx_pilot_cxdouble, rx_data_cxdouble);

    clear configs tx_pilot_cxdouble rx_pilot_cxdouble rx_data_cxdouble;

    %Plot Rx waveform
    for u = 1:total_users
        combined_rx = reshape(squeeze(rx_syms_cxdouble(:, u, :)), [], 1);
        figure('Name', ['User ' num2str(u) ', Frame ' num2str(inspect_frame) ' Receive WaveForm' ]);
        tiledlayout(2,1)
        %Top (Real)
        nexttile;
        plot(real(combined_rx));
        %axis([0 inf -1 1]);
        title('Rx Real (I)');
        %Bottom (Q)
        nexttile;
        plot(imag(combined_rx));
        %axis([0 inf -1 1]);
        title('Rx Imag (Q)');
        combined_tx = reshape(squeeze(tx_waveform(:, u, :)), [], 1);
        figure('Name', ['User ' num2str(u) ', Frame ' num2str(inspect_frame) ' Transmit WaveForm' ]);
        tiledlayout(2,1)
        %Top (Real)
        nexttile;
        plot(real(combined_tx));
        %axis([0 inf -1 1]);
        title('Tx Real (I)');
        %Bottom (Q)
        nexttile;
        plot(imag(combined_tx));
        %axis([0 inf -1 1]);
        title('Tx Imag (Q)');
    end

    for u=1:total_users
        rx_cnstl = demul_data(data_sc_idx, : , u);
        tx_cnstl = tx_data_cxdouble(data_sc_idx, u, :);
        figure('Name', ['Constellation [User ', num2str(u), ']']);
        pt_size = 15;
        scatter(real(rx_cnstl(:)), imag(rx_cnstl(:)),pt_size,'r','filled');
        hold on
        pt_size = 250;
        scatter(real(tx_cnstl(:)), imag(tx_cnstl(:)),pt_size, 'b', 'p', 'filled');
        title(['Constellation [User ', num2str(u), ', Frame ', num2str(inspect_frame), ']']);
    end
    clear total_users tx_data_cxdouble rx_beacon_cxdouble rx_syms_cxdouble tx_syms_cxdouble combined_rx;

    precision = 3;
    disp(['Frame Inspect: ', num2str(inspect_frame)]);
    disp(['Beacon RSSI (dB): ', mat2str(rx_beacon_rssi.', precision)]);
    disp(['Pilot SNR (dB): ', mat2str(rf_snr, precision)]);
    disp(['EVM SNR (dB): ', mat2str(snr, precision)]);
    disp(['EVM (%): ', mat2str(evm, precision)]);
end
