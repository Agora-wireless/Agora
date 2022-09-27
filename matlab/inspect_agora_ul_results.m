function inspect_agora_ul_results(dataset_filename, verbose)
    %%Load data from the input file and inspect evm and snr
    %dataset_filename = "BsRxData.h5";
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
    total_ul_symbols = double(h5readatt(dataset_filename, group_id, 'UL_SLOTS'));
    ul_pilot_symbols = double(h5readatt(dataset_filename, group_id, 'UL_PILOT_SLOTS'));
    %cl_sdr_id = h5readatt(dataset_filename, group_id, 'CL_SDR_ID');

    %Choose the Beacon data
    dataset_id = '/Pilot_Samples';

    %Display Info
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end
    %Generate a int16 array
    rx_pilots_hdf5 = h5read(dataset_filename, strcat(group_id,dataset_id));
    % Dimensions  [Samples, Ant, Symbol, Cells, Frame]
    total_users = size(rx_pilots_hdf5, 2);
    %Convert to double and scale
    rx_pilots_scaled_double = double(rx_pilots_hdf5) ./ double(intmax('int16'));
    clear rx_pilots_hdf5;
    %Convert to complex double
    % Samples x User x Symbol
    rx_pilots_cxdouble = complex(rx_pilots_scaled_double(1:2:end, :, :, 1:total_frames), rx_pilots_scaled_double(2:2:end,:, :, 1:total_frames));
    clear rx_pilots_scaled_double;

    %Choose the downlink data
    dataset_id = '/UplinkData';

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
    rx_pilot_cxdouble = rx_syms_cxdouble(:,:,1:ul_pilot_symbols, :);
    rx_data_cxdouble = rx_syms_cxdouble(:,:,1+ul_pilot_symbols:end, :);

    configs = [samples_per_slot tx_zero_prefix_len data_size data_start data_stop fft_size cp_len ...
    total_ul_symbols ul_pilot_symbols total_users];

    dataset_id = '/TxUplink';
    %*2 for complex type (native float)
    total_samples = data_size * 2;
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end 
    %start = [1 1 1 1 1];
    %count = [total_samples total_users 1 1 1];
    %tx_pilot_hdf5 = double(h5read(dataset_filename, strcat(group_id,dataset_id), start, count));
    %Convert to complex
    %tx_pilot_cxdouble = complex(tx_pilot_hdf5(1:2:end,:), tx_pilot_hdf5(2:2:end,:));
    %clear tx_pilot_hdf5 dataset_id start count;
end
