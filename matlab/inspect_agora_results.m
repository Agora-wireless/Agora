function inspect_agora_results(dataset_filename, inspect_frame, verbose)
    %%Load data from the input file and inspect evm and snr
    % -------- Fixed Values --------
    group_id = '/Data';
    %Display file / group attributes and datasets
    if verbose == "true"
        h5disp(dataset_filename,group_id);
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
    total_users = 1;
    
    configs = [samples_per_slot tx_zero_prefix_len data_size data_start data_stop fft_size cp_len ...
        total_dl_symbols dl_pilot_symbols total_users];

    %Choose the downlink data
    dataset_id = '/DownlinkData';

    % Dimensions  [Samples, Ant, Symbol, Cells, Frame]
    start = [1 1 1 1 inspect_frame];
    count = [(samples_per_slot * 2) total_users total_dl_symbols 1 1];
    %Display Info
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end
    %Generate a int16 array
    rx_syms_hdf5 = h5read(dataset_filename, strcat(group_id,dataset_id), start, count);
    %Convert to double and scale
    rx_syms_scaled_double = double(rx_syms_hdf5) ./ double(intmax('int16'));
    clear rx_syms_hdf5;
    %Convert to complex double
    % Samples x User x Symbol
    rx_syms_cxdouble = complex(rx_syms_scaled_double(1:2:end,:,:), rx_syms_scaled_double(2:2:end,:, :));
    clear rx_syms_scaled_double;
    % Split off pilots and data
    rx_pilot_cxdouble = rx_syms_cxdouble(:,:,1:dl_pilot_symbols);
    rx_data_cxdouble = rx_syms_cxdouble(:,:,1+dl_pilot_symbols:end);
    clear start count rx_syms_cxdouble;

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
    tx_data_cxdouble = complex(tx_data_hdf5(1:2:end,:,:), tx_data_hdf5(2:2:end,:,:));
    % Samples (complex) x User Ant x Downlink Symbol Id
    % removing the pilot for now because it doesn't check out?
    %tx_pilot_bad = tx_data_cxdouble(:,:,1);
    tx_data_cxdouble = tx_data_cxdouble(:,:,2:end);
    clear start count total_samples tx_data_hdf5 dataset_id;

    clear dataset_filename group_id;

    [evm, snr] = process_rx_frame(configs, tx_pilot_cxdouble, tx_data_cxdouble, rx_pilot_cxdouble, rx_data_cxdouble);

    clear u cl start_id rx_pilot_f_tmp rx_data_f_tmp plt_trk_sp nz_sc_idx;

    
    disp(['Frame Inspect: ', num2str(inspect_frame)]);
    disp(['SNR: ', num2str(snr)]);
    disp(['EVM: ', num2str(evm)]);
end
